#!/usr/bin/env python3

"""Efficient Docker resource monitoring using Linux cgroup v2.

CPU usage is reported as a percentage of one logical CPU.
100% therefore represents one fully utilized CPU core.

Memory is read from memory.current.
PIDs are read from pids.current (Linux tasks, including threads).
"""

import argparse
import math
import subprocess
import time

from pathlib import Path


class CgroupSampler:

    def __init__(self, containers):

        self.containers = list(containers)

        if not self.containers:
            raise ValueError("No containers supplied")

        if len(self.containers) != len(set(self.containers)):
            raise ValueError("Duplicate container names")

        self.paths = self.discover()

        # Initial counter readings establish the CPU baseline.
        self.previous = self.read_all()

    def discover(self):

        result = subprocess.run(
            [
                "docker",
                "inspect",
                "--format",
                "{{.Name}} {{.State.Pid}}",
                *self.containers,
            ],
            capture_output=True,
            text=True,
            check=True,
            timeout=15,
        )

        paths = {}

        for line in result.stdout.splitlines():

            name, pid_text = line.split()

            name = name.lstrip("/")
            pid = int(pid_text)

            if pid <= 0:
                raise RuntimeError(
                    f"{name}: container is not running"
                )

            entries = Path(
                f"/proc/{pid}/cgroup"
            ).read_text().splitlines()

            membership = next(
                (
                    line.split("::", 1)[1]
                    for line in entries
                    if line.startswith("0::")
                ),
                None,
            )

            if membership is None:
                raise RuntimeError(
                    f"{name}: cgroup v2 unavailable"
                )

            path = (
                Path("/sys/fs/cgroup")
                / membership.lstrip("/")
            )

            for filename in (
                "cpu.stat",
                "memory.current",
                "pids.current",
            ):

                if not (path / filename).is_file():
                    raise RuntimeError(
                        f"{name}: missing {filename}"
                    )

            paths[name] = path

        if set(paths) != set(self.containers):
            raise RuntimeError(
                "Container discovery incomplete"
            )

        return paths

    def read_one(self, path):

        started_ns = time.monotonic_ns()

        cpu_fields = dict(
            line.split()
            for line in (
                path / "cpu.stat"
            ).read_text().splitlines()
        )

        cpu_usage_usec = int(
            cpu_fields["usage_usec"]
        )

        memory_bytes = int(
            (path / "memory.current").read_text()
        )

        tasks = int(
            (path / "pids.current").read_text()
        )

        finished_ns = time.monotonic_ns()

        return {
            "cpu_usage_usec": cpu_usage_usec,
            "memory_bytes": memory_bytes,
            "pids": tasks,
            "timestamp_ns": (
                started_ns + finished_ns
            ) // 2,
        }

    def read_all(self):

        return {
            name: self.read_one(path)
            for name, path in self.paths.items()
        }

    def sample(self):

        collection_start_ns = time.monotonic_ns()

        current = self.read_all()

        collection_end_ns = time.monotonic_ns()

        measurements = {}

        for name in self.containers:

            previous = self.previous[name]
            latest = current[name]

            cpu_delta_usec = (
                latest["cpu_usage_usec"]
                - previous["cpu_usage_usec"]
            )

            elapsed_ns = (
                latest["timestamp_ns"]
                - previous["timestamp_ns"]
            )

            if elapsed_ns <= 0:
                raise RuntimeError(
                    f"{name}: invalid CPU interval"
                )

            if cpu_delta_usec < 0:
                raise RuntimeError(
                    f"{name}: CPU counter decreased"
                )

            elapsed_s = elapsed_ns / 1e9

            cpu_pct = (
                100
                * (cpu_delta_usec / 1e6)
                / elapsed_s
            )

            ram_mib = (
                latest["memory_bytes"] / 1024**2
            )

            if not all(
                math.isfinite(value)
                for value in (cpu_pct, ram_mib)
            ):
                raise RuntimeError(
                    f"{name}: invalid resource value"
                )

            measurements[name] = {
                "cpu_pct": cpu_pct,
                "ram_mib": ram_mib,
                "pids": latest["pids"],
                "cpu_interval_s": elapsed_s,
            }

        self.previous = current

        return {
            "measurements": measurements,
            "collection_start_ns": collection_start_ns,
            "collection_end_ns": collection_end_ns,
        }


def main():

    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "containers",
        nargs="+",
    )

    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
    )

    args = parser.parse_args()

    if (
        not math.isfinite(args.interval)
        or args.interval <= 0
    ):
        parser.error("Interval must be positive")

    print("===== INITIALIZING CGROUP MONITOR =====")

    monitor = CgroupSampler(args.containers)

    print(
        f"Containers discovered: "
        f"{len(monitor.containers)}"
    )

    print()
    print("===== WAITING FOR CPU MEASUREMENT INTERVAL =====")

    time.sleep(args.interval)

    result = monitor.sample()

    measurements = result["measurements"]

    print()
    print("===== RESOURCE MEASUREMENTS =====")

    for name, values in measurements.items():

        print()
        print(name)

        print(
            f"  CPU: {values['cpu_pct']:.2f}%"
        )

        print(
            f"  RAM: {values['ram_mib']:.2f} MiB"
        )

        print(
            f"  Tasks: {values['pids']}"
        )

        print(
            f"  CPU interval: "
            f"{values['cpu_interval_s']:.3f}s"
        )

    duration_ms = (
        result["collection_end_ns"]
        - result["collection_start_ns"]
    ) / 1e6

    print()
    print("===== COLLECTION PERFORMANCE =====")

    print(
        f"Cgroup snapshot duration: "
        f"{duration_ms:.3f} ms"
    )

    print()
    print("===== CGROUP MONITOR VALIDATION PASSED =====")


if __name__ == "__main__":
    main()
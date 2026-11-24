#!/usr/bin/env python3

"""Record fleet resource usage and Gazebo performance.

Initial idle-monitoring implementation.
Does not start, stop, or move simulated robots.
"""

import argparse
import csv
import json
import math
import subprocess
import threading
import time

from datetime import datetime, timezone
from pathlib import Path

from collect_resources import gpu_snapshot, memory_to_mib
from cgroup_resources import CgroupSampler
from run_baseline import read_gazebo


# Docker containers used by each architecture.

CORE = {
    "single": [
        "amr-scalability-simulation",
        "amr-scalability-localization",
        "amr-scalability-navigation",
    ],
    "fleet": [
        "amr-scalability-fleet-simulation",
        "amr-scalability-fleet-localization",
        "amr-scalability-fleet-navigation",
    ],
}

RMF = [
    "amr-scalability-rmf-schedule",
    "amr-scalability-rmf-dispatcher",
    "amr-scalability-rmf-fleet-adapter",
]


def command(args):
    result = subprocess.run(
        args,
        check=True,
        capture_output=True,
        text=True,
        timeout=15,
    )
    return result.stdout


def active_containers():
    output = command([
        "docker",
        "ps",
        "--format",
        "{{.Names}}",
    ])
    return set(output.splitlines())


def check_containers(layout, mode):
    expected = CORE[layout].copy()

    if mode == "rmf":
        expected.extend(RMF)

    active = active_containers()

    missing = sorted(set(expected) - active)

    unexpected = sorted(
        name
        for name in active
        if name.startswith("amr-scalability-")
        and name not in expected
    )

    if missing or unexpected:
        raise RuntimeError(
            f"Container preflight failed: "
            f"missing={missing}, unexpected={unexpected}"
        )

    return expected


def docker_snapshot(names):
    output = command([
        "docker",
        "stats",
        "--no-stream",
        "--format",
        "{{json .}}",
        *names,
    ])

    records = {
        item["Name"]: item
        for item in (
            json.loads(line)
            for line in output.splitlines()
        )
    }

    if set(records) != set(names):
        raise RuntimeError(
            "Docker resource snapshot incomplete"
        )

    measurements = {}

    for name in names:
        item = records[name]

        measurements[name] = {
            "cpu_pct": float(
                item["CPUPerc"].rstrip("%")
            ),
            "ram_mib": memory_to_mib(
                item["MemUsage"].split("/")[0].strip()
            ),
            "pids": int(item["PIDs"]),
        }

    return measurements


def host_snapshot():
    fields = (
        Path("/proc/stat")
        .read_text()
        .splitlines()[0]
        .split()
    )

    ticks = [int(value) for value in fields[1:]]

    cpu_total = sum(ticks)
    cpu_idle = ticks[3] + ticks[4]

    memory = {}

    for line in Path("/proc/meminfo").read_text().splitlines():
        key, _, value = line.partition(":")

        if key in ("MemTotal", "MemAvailable"):
            memory[key] = (
                int(value.strip().split()[0]) / 1024
            )

    return cpu_total, cpu_idle, memory


def write_csv(path, rows):
    if not rows:
        return

    with path.open("w", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=rows[0].keys(),
        )

        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--robots",
        type=int,
        required=True,
    )

    parser.add_argument(
        "--layout",
        choices=("single", "fleet"),
        required=True,
    )

    parser.add_argument(
        "--mode",
        choices=("nav2", "rmf"),
        required=True,
    )

    parser.add_argument(
        "--duration",
        type=int,
        default=20,
    )

    parser.add_argument(
        "--interval",
        type=float,
        default=3.0,
    )

    parser.add_argument(
        "--measurement-start-monotonic-ns",
        type=int,
        default=None,
        help=(
            "Shared CLOCK_MONOTONIC start timestamp "
            "provided by the benchmark controller"
        ),
    )

    args = parser.parse_args()

    if (
        args.measurement_start_monotonic_ns is not None
        and args.measurement_start_monotonic_ns <= 0
    ):
        parser.error(
            "--measurement-start-monotonic-ns "
            "must be a positive integer"
        )

    if (
        args.robots < 1
        or args.duration < 10
        or not math.isfinite(args.interval)
        or args.interval < 1
    ):
        parser.error(
            "Require robots >= 1, "
            "duration >= 10, interval >= 1"
        )

    if args.mode == "rmf" and args.layout != "fleet":
        parser.error(
            "RMF mode requires fleet layout"
        )

    if args.layout == "single" and args.robots != 1:
        parser.error(
            "Single layout supports only one robot"
        )

    names = check_containers(
        args.layout,
        args.mode,
    )

    timestamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    run_dir = (
        Path("results/raw/benchmark_v1")
        / (
            f"{args.robots}robots_"
            f"{args.mode}_idle_smoke_{timestamp}"
        )
    )

    run_dir.mkdir(
        parents=True,
        exist_ok=False,
    )

    print(
        f"Recording to: {run_dir}",
        flush=True,
    )

    container_rows = []
    host_rows = []
    rtf_rows = []
    errors = []

    discarded_late_samples = 0

    # Discover container cgroups and establish
    # the initial cumulative CPU counter readings.
    #
    # This happens once, before recording begins.

    sampler = CgroupSampler(names)

    # Use the controller's shared T0 when provided.
    # Otherwise, preserve standalone behavior.

    synchronized = (
        args.measurement_start_monotonic_ns is not None
    )

    if synchronized:
        start_ns = args.measurement_start_monotonic_ns
    else:
        start_ns = time.monotonic_ns()

    start = start_ns / 1e9

    end_ns = (
        start_ns + args.duration * 1_000_000_000
    )

    # Reject a synchronized experiment whose start
    # timestamp has already passed.

    if synchronized and time.monotonic_ns() >= start_ns:
        raise RuntimeError(
            "Shared measurement start has already passed"
        )

    start_utc = datetime.now(
        timezone.utc
    ).isoformat()

    # In synchronized mode, Gazebo monitoring starts
    # before T0 so the subscription is already active.
    #
    # Extend the subprocess timeout to cover startup
    # plus the complete measurement window.

    gazebo_timeout_s = float(args.duration)

    if synchronized:
        gazebo_timeout_s += (
            max(0.0, start - time.monotonic())
            + 3.0
        )
    gazebo_command = [
        "docker",
        "exec",
        CORE[args.layout][0],
        "bash",
        "-lc",
        (
            "source /opt/ros/jazzy/setup.bash && "
            f"timeout {gazebo_timeout_s:.3f}s "
            "gz topic -e -t /world/warehouse/stats"
        ),
    ]

    with (
        run_dir / "gazebo_stderr.log"
    ).open("w") as log:

        process = subprocess.Popen(
            gazebo_command,
            stdout=subprocess.PIPE,
            stderr=log,
            text=True,
            bufsize=1,
        )

        reader = threading.Thread(
            target=read_gazebo,
            args=(
                process.stdout,
                start,
                rtf_rows,
            ),
            daemon=True,
        )

        reader.start()

        index = 0

        try:

            if synchronized:

                # Inform the controller that the
                # recording infrastructure is initialized.

                if time.monotonic_ns() >= start_ns:
                    raise RuntimeError(
                        "Resource recorder initialized "
                        "after the shared measurement start"
                    )

                print(
                    "RESOURCE_RECORDER_READY "
                    f"measurement_start_ns={start_ns}",
                    flush=True,
                )

                # Wait until the shared measurement
                # window begins.

                remaining = (
                    start - time.monotonic()
                )

                if remaining > 0:
                    time.sleep(remaining)

                # Reset cumulative CPU counters at T0.
                #
                # This prevents pre-experiment activity
                # from affecting the first CPU sample.

                sampler.previous = sampler.read_all()

            # Establish the host CPU baseline.
            # In synchronized mode this occurs at T0.

            prev_total, prev_idle, _ = host_snapshot()

            if synchronized:
                start_utc = datetime.now(
                    timezone.utc
                ).isoformat()

            while (
                time.monotonic_ns() < end_ns
            ):

                # First sample occurs after one complete
                # CPU observation interval.
                #
                # For a 3-second interval:
                # samples occur at 3, 6, 9, 12, 15, 18...

                target = start + (index + 1) * args.interval

                remaining = target - time.monotonic()

                if remaining > 0:
                    time.sleep(remaining)

                if (
                    time.monotonic() - start
                    >= args.duration
                ):
                    break

                timestamp_ns = time.time_ns()

                try:
                    # Docker resource collection interval
                    docker_start_ns = time.monotonic_ns()

                    # Read CPU, RAM, and task counts
                    # directly from Linux cgroup v2.

                    resource_result = sampler.sample()

                    docker_data = resource_result["measurements"]

                    docker_end_ns = time.monotonic_ns()

                    # NVIDIA GPU collection interval
                    gpu_start_ns = time.monotonic_ns()

                    gpu_data = gpu_snapshot()

                    gpu_end_ns = time.monotonic_ns()

                    # Host CPU and memory measurement
                    total, idle, memory = host_snapshot()

                    host_snapshot_ns = time.monotonic_ns()

                    # The entire resource measurement must finish
                    # inside the requested experiment window.
                    #
                    # If Docker/GPU/host collection finishes after
                    # the deadline, discard the complete sample group.

                    sample_end_elapsed_s = (
                        host_snapshot_ns / 1e9 - start
                    )

                    if sample_end_elapsed_s >= args.duration:

                        discarded_late_samples += 1

                        print(
                            f"Sample {index + 1} discarded: "
                            f"collection finished at "
                            f"{sample_end_elapsed_s:.3f}s "
                            f"(deadline={args.duration}s)",
                            flush=True,
                        )

                        break

                    delta_total = total - prev_total

                    host_cpu = (
                        100 * (
                            1 - (
                                (idle - prev_idle)
                                / delta_total
                            )
                        )
                        if delta_total > 0
                        else None
                    )

                    prev_total = total
                    prev_idle = idle

                    for name in names:
                        container_rows.append({
                            "sample_id": index,
                            "time_utc_ns": timestamp_ns,
                            "phase": "idle",
                            "container": name,

                            "collection_start_elapsed_s": round(
                                docker_start_ns / 1e9 - start,
                                6,
                            ),

                            "collection_end_elapsed_s": round(
                                docker_end_ns / 1e9 - start,
                                6,
                            ),

                            "collection_duration_s": round(
                                (
                                    docker_end_ns
                                    - docker_start_ns
                                ) / 1e9,
                                6,
                            ),

                            **docker_data[name],
                        })

                    host_rows.append({
                        "sample_id": index,
                        "time_utc_ns": timestamp_ns,
                        "phase": "idle",

                        "gpu_start_elapsed_s": round(
                            gpu_start_ns / 1e9 - start,
                            6,
                        ),

                        "gpu_end_elapsed_s": round(
                            gpu_end_ns / 1e9 - start,
                            6,
                        ),

                        "gpu_collection_duration_s": round(
                            (
                                gpu_end_ns
                                - gpu_start_ns
                            ) / 1e9,
                            6,
                        ),

                        "host_snapshot_elapsed_s": round(
                            host_snapshot_ns / 1e9 - start,
                            6,
                        ),

                        "host_cpu_pct": host_cpu,

                        "host_ram_used_mib": (
                            memory["MemTotal"]
                            - memory["MemAvailable"]
                        ),

                        **gpu_data,
                    })

                    print(
                        f"Sample {index + 1}: "
                        f"{len(names)} containers, "
                        f"RTF samples={len(rtf_rows)}",
                        flush=True,
                    )

                except Exception as exc:
                    errors.append(
                        f"Sample {index}: {exc}"
                    )
                    break

                index += 1

        finally:
            try:
                process.wait(
                    timeout=args.duration + 10
                )

            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=10)

            reader.join(timeout=5)
            process.stdout.close()

    # Keep only measurements inside the requested window.
    # This prevents delayed samples from contaminating
    # comparisons between different fleet sizes.

    rtf_rows = [
        row
        for row in rtf_rows
        if 0.0 <= row["received_elapsed_s"] < args.duration
    ]

    write_csv(
        run_dir / "containers.csv",
        container_rows,
    )

    write_csv(
        run_dir / "host_gpu.csv",
        host_rows,
    )

    write_csv(
        run_dir / "gazebo_stats.csv",
        rtf_rows,
    )

    passed = (
        not errors
        and len(host_rows) >= 3
        and len(rtf_rows) >= 3
    )

    validation = {
        "status": (
            "resource_smoke_pass"
            if passed
            else "incomplete"
        ),
        "errors": errors,
        "resource_samples": len(host_rows),
        "per_container_samples": len(container_rows),
        "gazebo_rtf_samples": len(rtf_rows),

        "discarded_late_resource_samples":
            discarded_late_samples,
    }

    metadata = {
        "protocol": "benchmark-v1-monitor-smoke",

        "synchronized": synchronized,

        "measurement_start_monotonic_ns": start_ns,

        "measurement_end_monotonic_ns": end_ns,

        "measurement_clock": "CLOCK_MONOTONIC",

        "resource_backend": "cgroup_v2",

        "cpu_measurement": (
            "Delta of cumulative cgroup CPU usage "
            "over the observed monotonic interval"
        ),

        "cpu_normalization": (
            "100 percent represents one logical CPU"
        ),

        "memory_measurement": (
            "cgroup v2 memory.current"
        ),

        "task_measurement": (
            "cgroup v2 pids.current"
        ),

        "start_utc": start_utc,
        "duration_requested_s": args.duration,
        "interval_requested_s": args.interval,
        "robot_count_declared_unverified": args.robots,
        "mode": args.mode,
        "layout": args.layout,
        "phase": "idle",
        "containers": names,
        "validation": validation,
        "note": (
            "Idle monitoring smoke test. "
            "Not a complete scalability benchmark."
        ),
    }

    (
        run_dir / "metadata.json"
    ).write_text(
        json.dumps(metadata, indent=2) + "\n"
    )

    print()
    print("===== RECORDING RESULTS =====")
    print("Status:", validation["status"])
    print("Resource samples:", len(host_rows))
    print("RTF samples:", len(rtf_rows))
    print("Output:", run_dir)

    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
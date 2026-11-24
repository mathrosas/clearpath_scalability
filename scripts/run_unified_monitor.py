#!/usr/bin/env python3
"""Run a synchronized idle resource + RMF monitoring smoke test.

Does not dispatch tasks, move robots, or change the running stack.
"""

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
import time

from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent

BASE = ROOT / "results/raw/benchmark_v1"


def csv_rows(path):

    with path.open(
        newline="",
        encoding="utf-8",
    ) as handle:

        return list(csv.DictReader(handle))


def validate(run_dir, experiment_id, t0, t1, robots):

    resource = run_dir / "resource"

    meta = json.loads(
        (resource / "metadata.json").read_text()
    )

    containers = csv_rows(
        resource / "containers.csv"
    )

    host = csv_rows(
        resource / "host_gpu.csv"
    )

    gazebo = csv_rows(
        resource / "gazebo_stats.csv"
    )

    events = [
        json.loads(line)
        for line in (
            run_dir / "rmf_events.jsonl"
        ).read_text().splitlines()
        if line.strip()
    ]

    measured = [
        event
        for event in events
        if event["phase"] == "measurement"
    ]

    robots_observed = {
        robot["name"]
        for event in measured
        if event["topic"] == "/fleet_states"
        for robot in event["payload"].get(
            "robots", []
        )
    }

    duration = (t1 - t0) / 1e9

    expected_robots = {
        f"j100_{i:04d}"
        for i in range(robots)
    }

    checks = {
        "resource_shared_t0": (
            meta.get(
                "measurement_start_monotonic_ns"
            ) == t0
        ),

        "resource_shared_t1": (
            meta.get(
                "measurement_end_monotonic_ns"
            ) == t1
        ),

        "resource_synchronized": (
            meta.get("synchronized") is True
        ),

        "resource_validation": (
            meta.get(
                "validation", {}
            ).get("status") == "resource_smoke_pass"
        ),

        "container_samples": (
            len(containers) > 0
        ),

        "host_gpu_samples": (
            len(host) >= 3
        ),

        "gazebo_samples": (
            len(gazebo) >= 3
        ),

        "resource_times_in_window": all(
            0 <= float(
                row["collection_start_elapsed_s"]
            )
            <= float(
                row["collection_end_elapsed_s"]
            )
            < duration
            for row in containers
        ),

        "gazebo_times_in_window": all(
            0 <= float(
                row["received_elapsed_s"]
            ) < duration
            for row in gazebo
        ),

        "rmf_events_received": (
            len(events) > 0
        ),

        "rmf_experiment_id": all(
            event["experiment_id"] == experiment_id
            for event in events
        ),

        "rmf_measurement_events": (
            len(measured) > 0
        ),

        "rmf_events_in_window": all(
            t0 <= event["received_monotonic_ns"] < t1
            for event in measured
        ),

        "rmf_startup_before_t0": all(
            event["received_monotonic_ns"] < t0
            for event in events
            if event["phase"] == "startup"
        ),

        "rmf_no_late_events": all(
            event["phase"] != "outside_window"
            for event in events
        ),

        "robots_observed": (
            expected_robots.issubset(
                robots_observed
            )
        ),
    }

    print(
        f"Container measurements: "
        f"{len(containers)}"
    )

    print(
        f"Host/GPU samples: {len(host)}"
    )

    print(
        f"Gazebo samples: {len(gazebo)}"
    )

    print(
        f"RMF events: {len(events)} "
        f"({len(measured)} during measurement)"
    )

    for name, passed in checks.items():

        print(
            f"{name}: "
            f"{'PASS' if passed else 'FAIL'}"
        )

    return checks


def stop_processes(processes):

    for process in processes:

        if process.poll() is None:
            process.terminate()

    for process in processes:

        if process.poll() is None:

            try:
                process.wait(timeout=5)

            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)


def main():

    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--robots",
        type=int,
        default=2,
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
        "--lead-seconds",
        type=float,
        default=25.0,
    )

    args = parser.parse_args()

    if args.robots < 1 or args.duration < 10:

        parser.error(
            "Require robots >= 1 and duration >= 10"
        )

    if (
        not math.isfinite(args.interval)
        or args.interval < 1
    ):

        parser.error(
            "Require interval >= 1"
        )

    if (
        not math.isfinite(args.lead_seconds)
        or args.lead_seconds < 15
    ):

        parser.error(
            "Require lead-seconds >= 15"
        )

    stamp = datetime.now(
        timezone.utc
    ).strftime(
        "%Y%m%dT%H%M%S%fZ"
    )

    experiment_id = (
        f"unified-{args.robots}robots-"
        f"rmf-idle-{stamp}"
    )

    run_dir = BASE / experiment_id

    run_dir.mkdir(
        parents=True,
        exist_ok=False,
    )

    # Establish one shared monotonic time window.

    t0 = (
        time.monotonic_ns()
        + int(args.lead_seconds * 1e9)
    )

    t1 = (
        t0
        + args.duration * 1_000_000_000
    )

    report = {
        "protocol": "benchmark-v1-unified-idle-smoke",

        "experiment_id": experiment_id,

        "state": "starting",

        "robot_count_declared_unverified":
            args.robots,

        "duration_s": args.duration,

        "interval_s": args.interval,

        "measurement_start_monotonic_ns": t0,

        "measurement_end_monotonic_ns": t1,

        "created_utc": datetime.now(
            timezone.utc
        ).isoformat(),

        "artifacts": {
            "resource": "resource",
            "rmf_events": "rmf_events.jsonl",
            "resource_log": "resource.log",
            "rmf_log": "rmf.log",
        },

        "note": (
            "Idle monitoring only, "
            "not a task-completion benchmark"
        ),
    }

    # Resource recorder executes on Ubuntu.

    resource_cmd = [
        sys.executable,
        "-u",
        str(
            ROOT
            / "scripts/record_scalability_perf.py"
        ),

        "--robots",
        str(args.robots),

        "--layout",
        "fleet",

        "--mode",
        "rmf",

        "--duration",
        str(args.duration),

        "--interval",
        str(args.interval),

        "--measurement-start-monotonic-ns",
        str(t0),
    ]

    # RMF observer executes inside the fleet adapter.
    # The host and container share the validated
    # CLOCK_MONOTONIC reference.

    rmf_command = (
        "source /opt/ros/jazzy/setup.bash && "
        "exec python3 "
        "/opt/amr/scripts/record_rmf_events.py "
        '--experiment-id "$1" '
        '--duration "$2" '
        '--measurement-start-monotonic-ns "$3"'
    )

    rmf_cmd = [
        "docker",
        "exec",
        "amr-scalability-rmf-fleet-adapter",

        "bash",
        "-lc",
        rmf_command,

        "_",
        experiment_id,
        str(args.duration),
        str(t0),
    ]

    processes = []

    print(
        "===== UNIFIED RMF IDLE MONITORING =====",
        flush=True,
    )

    print(
        f"Experiment: {experiment_id}"
    )

    print(
        f"T0: {t0}"
    )

    print(
        f"T1: {t1}"
    )

    print(
        f"Output: {run_dir}",
        flush=True,
    )

    try:

        with (
            (
                run_dir / "resource.log"
            ).open(
                "w",
                encoding="utf-8",
            ) as resource_log,

            (
                run_dir / "rmf.log"
            ).open(
                "w",
                encoding="utf-8",
            ) as rmf_log,

            (
                run_dir / "rmf_events.jsonl"
            ).open(
                "w",
                encoding="utf-8",
            ) as rmf_events,
        ):

            # Launch both recorders.

            resource_proc = subprocess.Popen(
                resource_cmd,
                cwd=ROOT,
                stdout=resource_log,
                stderr=subprocess.STDOUT,
            )

            processes.append(
                resource_proc
            )

            rmf_proc = subprocess.Popen(
                rmf_cmd,
                cwd=ROOT,
                stdout=rmf_events,
                stderr=rmf_log,
            )

            processes.append(
                rmf_proc
            )

            # Both recorders report initialization
            # after their respective setup routines.
            #
            # Require exact timing agreement before T0.

            required = (
                (
                    run_dir / "resource.log",
                    (
                        "RESOURCE_RECORDER_READY "
                        f"measurement_start_ns={t0}"
                    ),
                ),

                (
                    run_dir / "rmf.log",
                    "RMF observer initialized",
                ),

                (
                    run_dir / "rmf.log",
                    (
                        "Measurement start "
                        f"monotonic_ns: {t0}"
                    ),
                ),

                (
                    run_dir / "rmf.log",
                    (
                        "Measurement end "
                        f"monotonic_ns: {t1}"
                    ),
                ),
            )

            ready = False

            # Require readiness at least two seconds
            # before the shared experiment starts.

            while (
                time.monotonic_ns()
                < t0 - 2_000_000_000
            ):

                if all(
                    marker in path.read_text()
                    for path, marker in required
                ):

                    ready = True
                    break

                if any(
                    process.poll() is not None
                    for process in processes
                ):

                    raise RuntimeError(
                        "Recorder exited before readiness"
                    )

                time.sleep(0.2)

            if not ready:

                raise RuntimeError(
                    "Recorders were not ready "
                    "at least 2 seconds before T0"
                )

            print(
                "BOTH RECORDERS READY BEFORE T0: PASS",
                flush=True,
            )

            report["ready_before_t0"] = True

            # Permit the recorders to complete their
            # measurement and shutdown procedures.

            deadline = (
                t1 + 25_000_000_000
            )

            while any(
                process.poll() is None
                for process in processes
            ):

                if time.monotonic_ns() >= deadline:

                    raise RuntimeError(
                        "Recorder completion "
                        "deadline exceeded"
                    )

                if any(
                    process.poll() not in (None, 0)
                    for process in processes
                ):

                    raise RuntimeError(
                        "A recorder exited unsuccessfully"
                    )

                time.sleep(0.2)

            if any(
                process.returncode != 0
                for process in processes
            ):

                raise RuntimeError(
                    "Recorder exit codes: "
                    f"{[p.returncode for p in processes]}"
                )

        # Consolidate the resource recorder's
        # standalone output under this experiment.

        resource_log_text = (
            run_dir / "resource.log"
        ).read_text()

        datasets = [
            line.removeprefix(
                "Recording to: "
            ).strip()

            for line in resource_log_text.splitlines()

            if line.startswith(
                "Recording to: "
            )
        ]

        if len(datasets) != 1:

            raise RuntimeError(
                "Could not identify resource dataset: "
                f"{datasets}"
            )

        old_resource = (
            ROOT / datasets[0]
        ).resolve()

        if (
            old_resource.parent != BASE.resolve()
            or not old_resource.is_dir()
        ):

            raise RuntimeError(
                "Unexpected resource output path: "
                f"{old_resource}"
            )

        shutil.move(
            str(old_resource),
            str(run_dir / "resource"),
        )

        # Validate both datasets against the same
        # experiment ID and monotonic time window.

        report["checks"] = validate(
            run_dir,
            experiment_id,
            t0,
            t1,
            args.robots,
        )

        if not all(
            report["checks"].values()
        ):

            raise RuntimeError(
                "Unified data-integrity "
                "validation failed"
            )

        report["state"] = "passed"

        print(
            "UNIFIED IDLE MONITORING: PASSED"
        )

    except Exception as exc:

        report["state"] = "failed"

        report["error"] = str(exc)

        print(
            "UNIFIED IDLE MONITORING: "
            f"FAILED ({exc})",
            file=sys.stderr,
        )

        raise SystemExit(1)

    finally:

        stop_processes(processes)

        report["recorder_exit_codes"] = [
            process.returncode
            for process in processes
        ]

        report["finished_utc"] = (
            datetime.now(
                timezone.utc
            ).isoformat()
        )

        (
            run_dir / "controller_metadata.json"
        ).write_text(
            json.dumps(
                report,
                indent=2,
            ) + "\n",
            encoding="utf-8",
        )

        print(
            f"Dataset: {run_dir}"
        )


if __name__ == "__main__":
    main()
#!/usr/bin/env python3

"""Passive Open-RMF event recorder for AMR scalability benchmarks.

Writes JSONL events to stdout and diagnostics to stderr.

Run inside a ROS 2 environment with Open-RMF message packages.
This program does not publish messages or control robots.
"""

import argparse
import json
import math
import sys
import time

from collections import Counter

import rclpy

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from rosidl_runtime_py.convert import message_to_ordereddict

from rmf_task_msgs.msg import TaskSummary, DispatchStates
from rmf_fleet_msgs.msg import FleetState

from rmf_traffic_msgs.msg import (
    Participants,
    ItinerarySet,
    NegotiationNotice,
    NegotiationConclusion,
)


# These QoS settings reflect the publishers inspected
# in the running two-robot Open-RMF environment.

TOPICS = [
    (
        "/task_summaries",
        TaskSummary,
        "task_summary",
        True,
    ),
    (
        "/dispatch_states",
        DispatchStates,
        "dispatch_state",
        False,
    ),
    (
        "/fleet_states",
        FleetState,
        "fleet_state",
        True,
    ),
    (
        "/rmf_traffic/participants",
        Participants,
        "participants",
        True,
    ),
    (
        "/rmf_traffic/itinerary_set",
        ItinerarySet,
        "itinerary_set",
        True,
    ),
    (
        "/rmf_traffic/negotiation_notice",
        NegotiationNotice,
        "negotiation_notice",
        False,
    ),
    (
        "/rmf_traffic/negotiation_conclusion",
        NegotiationConclusion,
        "negotiation_conclusion",
        False,
    ),
]


def make_qos(transient_local):

    return QoSProfile(
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient_local
            else DurabilityPolicy.VOLATILE
        ),
    )


class RMFEventRecorder:

    def __init__(
        self,
        node,
        experiment_id,
        startup_seconds,
        duration,
        measurement_start_monotonic_ns=None,
    ):

        self.node = node
        self.experiment_id = experiment_id
        self.duration = duration

        self.counts = Counter()
        self.phase_counts = Counter()

        self.subscriptions = []

        self.created_ns = time.monotonic_ns()

        self.external_start = (
            measurement_start_monotonic_ns is not None
        )

        if self.external_start:

            # Shared experiment clock supplied by
            # the benchmark controller.

            self.measurement_start_ns = int(
                measurement_start_monotonic_ns
            )

        else:

            # Preserve standalone recording behavior.

            self.measurement_start_ns = (
                self.created_ns
                + int(startup_seconds * 1e9)
            )

        self.measurement_end_ns = (
            self.measurement_start_ns
            + int(duration * 1e9)
        )

        for (
            topic,
            message_type,
            event_type,
            transient_local,
        ) in TOPICS:

            callback = self.make_callback(
                topic,
                event_type,
            )

            subscription = node.create_subscription(
                message_type,
                topic,
                callback,
                make_qos(transient_local),
            )

            self.subscriptions.append(subscription)

    def make_callback(self, topic, event_type):

        def callback(message):

            received_monotonic_ns = time.monotonic_ns()
            received_utc_ns = time.time_ns()

            if (
                received_monotonic_ns
                < self.measurement_start_ns
            ):
                phase = "startup"

            elif (
                received_monotonic_ns
                < self.measurement_end_ns
            ):
                phase = "measurement"

            else:
                phase = "outside_window"

            elapsed_s = (
                received_monotonic_ns
                - self.measurement_start_ns
            ) / 1e9

            event = {
                "schema_version": 1,
                "experiment_id": self.experiment_id,
                "topic": topic,
                "event_type": event_type,
                "phase": phase,
                "received_utc_ns": received_utc_ns,
                "received_monotonic_ns":
                    received_monotonic_ns,
                "elapsed_s": round(elapsed_s, 9),
                "payload": message_to_ordereddict(
                    message
                ),
            }

            # stdout contains only JSONL records.
            # Flush each event to reduce data loss
            # if the recording is interrupted.

            sys.stdout.write(
                json.dumps(
                    event,
                    separators=(",", ":"),
                    allow_nan=False,
                )
                + "\n"
            )

            sys.stdout.flush()

            self.counts[topic] += 1
            self.phase_counts[phase] += 1

        return callback

    def run(self):

        # A synchronized recorder must be ready
        # before the shared measurement window begins.

        if (
            self.external_start
            and time.monotonic_ns()
            >= self.measurement_start_ns
        ):
            raise RuntimeError(
                "Shared measurement start has already passed. "
                "Observer initialization was too late."
            )

        print(
            "RMF observer initialized",
            file=sys.stderr,
            flush=True,
        )

        print(
            f"Experiment: {self.experiment_id}",
            file=sys.stderr,
            flush=True,
        )

        print(
            f"Subscriptions: {len(self.subscriptions)}",
            file=sys.stderr,
            flush=True,
        )

        print(
            "Measurement start monotonic_ns: "
            f"{self.measurement_start_ns}",
            file=sys.stderr,
            flush=True,
        )

        print(
            "Measurement end monotonic_ns: "
            f"{self.measurement_end_ns}",
            file=sys.stderr,
            flush=True,
        )

        try:

            while True:

                remaining_ns = (
                    self.measurement_end_ns
                    - time.monotonic_ns()
                )

                if remaining_ns <= 0:
                    break

                rclpy.spin_once(
                    self.node,
                    timeout_sec=min(
                        0.2,
                        remaining_ns / 1e9,
                    ),
                )

        except KeyboardInterrupt:

            print(
                "Observer interrupted",
                file=sys.stderr,
                flush=True,
            )

            raise

        finally:

            print(
                "\n===== RMF EVENT SUMMARY =====",
                file=sys.stderr,
            )

            for topic, _, _, _ in TOPICS:

                print(
                    f"{topic}: {self.counts[topic]}",
                    file=sys.stderr,
                )

            print(
                "Phase counts: "
                f"{dict(self.phase_counts)}",
                file=sys.stderr,
            )

            print(
                "===== OBSERVER FINISHED =====",
                file=sys.stderr,
                flush=True,
            )


def main():

    parser = argparse.ArgumentParser(
        description=__doc__,
    )

    parser.add_argument(
        "--experiment-id",
        required=True,
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Measurement duration in seconds",
    )

    parser.add_argument(
        "--startup-seconds",
        type=float,
        default=2.0,
        help="Initial subscription synchronization period",
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
        not math.isfinite(args.duration)
        or args.duration <= 0
    ):
        parser.error(
            "--duration must be finite and positive"
        )

    if (
        not math.isfinite(args.startup_seconds)
        or args.startup_seconds < 0
    ):
        parser.error(
            "--startup-seconds must be finite and nonnegative"
        )

    if (
        args.measurement_start_monotonic_ns is not None
        and args.measurement_start_monotonic_ns <= 0
    ):
        parser.error(
            "--measurement-start-monotonic-ns "
            "must be a positive integer"
        )

    rclpy.init()

    node = None

    try:

        node = rclpy.create_node(
            "rmf_benchmark_event_recorder"
        )

        recorder = RMFEventRecorder(
            node=node,
            experiment_id=args.experiment_id,
            startup_seconds=args.startup_seconds,
            duration=args.duration,
            measurement_start_monotonic_ns=(
                args.measurement_start_monotonic_ns
            ),
        )

        recorder.run()

    finally:

        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3

"""Submit a patrol task through the Open-RMF JSON task API.

Default mode validates and prints the request without publishing.
Use --execute to submit exactly one task.
"""

import argparse
import json
import time
import uuid

from pathlib import Path


GRAPH_PATH = Path("/opt/amr/rmf/nav_graph.yaml")

FLEET_NAME = "j100_fleet"


def load_waypoints():

    import yaml

    with GRAPH_PATH.open(
        "r",
        encoding="utf-8",
    ) as file:

        graph = yaml.safe_load(file)

    vertices = graph["levels"]["warehouse"]["vertices"]

    names = {
        vertex[2]["name"]
        for vertex in vertices
        if vertex[2].get("name")
    }

    return names


def build_request(places, rounds):

    return {
        "type": "dispatch_task_request",

        "request": {
            "category": "patrol",

            "description": {
                "places": places,
                "rounds": rounds,
            },

            "fleet_name": FLEET_NAME,

            "requester": "amr_scalability_benchmark",
        },
    }


def submit_request(payload, request_id, timeout):

    import rclpy

    from rmf_task_msgs.msg import (
        ApiRequest,
        ApiResponse,
    )

    from rclpy.qos import (
        QoSProfile,
        ReliabilityPolicy,
        DurabilityPolicy,
    )

    rclpy.init()

    node = rclpy.create_node(
        "amr_scalability_task_probe"
    )

    transient_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    response_qos = QoSProfile(
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    responses = []

    def on_response(message):

        if message.request_id != request_id:
            return

        if message.type != ApiResponse.TYPE_RESPONDING:
            return

        try:
            response = json.loads(message.json_msg)
        except json.JSONDecodeError as error:
            response = {
                "success": False,
                "error": f"Invalid response JSON: {error}",
            }

        responses.append(response)

    publisher = node.create_publisher(
        ApiRequest,
        "/task_api_requests",
        transient_qos,
    )

    subscription = node.create_subscription(
        ApiResponse,
        "/task_api_responses",
        on_response,
        response_qos,
    )

    try:

        print(
            "\n===== WAITING FOR RMF DISPATCHER =====",
            flush=True,
        )

        deadline = time.monotonic() + timeout

        while publisher.get_subscription_count() == 0:

            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "No RMF task API subscribers discovered"
                )

            rclpy.spin_once(
                node,
                timeout_sec=0.2,
            )

        print(
            "Task API subscriber discovered",
            flush=True,
        )

        request = ApiRequest()

        request.request_id = request_id

        request.json_msg = json.dumps(
            payload,
            separators=(",", ":"),
        )

        print(
            "\n===== SUBMITTING RMF TASK =====",
            flush=True,
        )

        publisher.publish(request)

        print(
            f"Request ID: {request_id}",
            flush=True,
        )

        deadline = time.monotonic() + timeout

        while not responses:

            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "RMF task API response timed out. "
                    "The task may still have been submitted; "
                    "do not retry without checking RMF state."
                )

            rclpy.spin_once(
                node,
                timeout_sec=0.2,
            )

        response = responses[0]

        print(
            "\n===== RMF TASK API RESPONSE =====",
            flush=True,
        )

        print(
            json.dumps(
                response,
                indent=2,
            ),
            flush=True,
        )

        if response.get("success") is not True:
            raise RuntimeError(
                "RMF rejected the task request"
            )

        print(
            "\nRMF REQUEST ACCEPTED",
            flush=True,
        )

        print(
            "Task execution and completion "
            "have NOT yet been verified.",
            flush=True,
        )

    finally:

        node.destroy_subscription(subscription)
        node.destroy_publisher(publisher)

        node.destroy_node()

        rclpy.shutdown()


def main():

    parser = argparse.ArgumentParser(
        description=__doc__,
    )

    parser.add_argument(
        "--places",
        nargs="+",
        default=["east_goal"],
        help="Named RMF navigation waypoints",
    )

    parser.add_argument(
        "--rounds",
        type=int,
        default=1,
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually submit the task to Open-RMF",
    )

    args = parser.parse_args()

    if args.rounds < 1:
        parser.error("--rounds must be >= 1")

    if not 1 <= args.timeout <= 60:
        parser.error(
            "--timeout must be between 1 and 60 seconds"
        )

    available = load_waypoints()

    unknown = set(args.places) - available

    if unknown:
        parser.error(
            f"Unknown RMF waypoints: {sorted(unknown)}"
        )

    payload = build_request(
        args.places,
        args.rounds,
    )

    request_id = (
        f"amr-patrol-{uuid.uuid4().hex}"
    )

    print(
        "===== OPEN-RMF PATROL REQUEST ====="
    )

    print(
        f"Fleet: {FLEET_NAME}"
    )

    print(
        f"Request ID: {request_id}"
    )

    print(
        f"Waypoints: {args.places}"
    )

    print(
        f"Rounds: {args.rounds}"
    )

    print(
        "\n===== JSON REQUEST ====="
    )

    print(
        json.dumps(
            payload,
            indent=2,
        )
    )

    if not args.execute:

        print(
            "\nDRY RUN: PASSED"
        )

        print(
            "No task was submitted."
        )

        return

    submit_request(
        payload,
        request_id,
        args.timeout,
    )


if __name__ == "__main__":
    main()
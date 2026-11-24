#!/usr/bin/env python3

"""Open-RMF fleet adapter for the Clearpath J100 fleet.

Stage 1:
    - Load the existing fleet configuration.
    - Initialize Open-RMF.
    - Create one Nav2 client per robot.
    - Wait for valid AMCL poses and available Nav2 action servers.
    - Register robots with Open-RMF.
    - Publish periodic robot state updates.

Navigation execution is supported through the Nav2 action interface.
"""

import argparse
import json
import tempfile

from pathlib import Path

import numpy as np
import rclpy
import rmf_adapter
import rmf_adapter.easy_full_control as rmf_easy
import yaml
import threading

from rmf_nav2_client import Nav2RobotClient


MAP_NAME = "warehouse"
STATE_UPDATE_PERIOD = 0.5
SIMULATED_BATTERY_SOC = 1.0


def load_yaml(path):
    """Load a YAML configuration file."""

    with Path(path).open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def build_rmf_configuration(
    fleet_path,
    rmf_path,
    graph_path,
):
    """Build an RMF configuration using the existing fleet membership."""

    fleet_data = load_yaml(fleet_path)
    rmf_data = load_yaml(rmf_path)
    graph_data = load_yaml(graph_path)

    robots = fleet_data["robots"]

    robot_names = [
        robot["namespace"]
        for robot in robots
    ]

    if not robot_names:
        raise ValueError("Fleet configuration contains no robots")

    if len(robot_names) != len(set(robot_names)):
        raise ValueError("Duplicate robot namespaces")

    if fleet_data["world"] != MAP_NAME:
        raise ValueError(
            f"Unexpected fleet world: {fleet_data['world']}"
        )

    if MAP_NAME not in graph_data["levels"]:
        raise ValueError(
            f"Map {MAP_NAME} is missing from RMF graph"
        )

    # Robot membership is defined only in config/fleet/fleet.yaml.
    rmf_data["rmf_fleet"]["robots"] = {
        name: None
        for name in robot_names
    }

    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".yaml",
        encoding="utf-8",
    ) as temporary_config:

        yaml.safe_dump(
            rmf_data,
            temporary_config,
            sort_keys=False,
        )

        temporary_config.flush()

        configuration = (
            rmf_easy.FleetConfiguration.from_config_files(
                temporary_config.name,
                str(graph_path),
            )
        )

    if configuration is None:
        raise RuntimeError(
            "Open-RMF rejected the fleet configuration"
        )

    if sorted(configuration.known_robots) != sorted(robot_names):
        raise RuntimeError(
            "RMF robot membership does not match fleet.yaml"
        )

    return configuration, robot_names


class RMFFleetBridge:
    """Manage RMF registration and state reporting."""

    def __init__(
        self,
        configuration,
        robot_names,
    ):

        self.configuration = configuration
        self.robot_names = robot_names

        self.clients = {}
        self.update_handles = {}

        self._execution_lock = threading.RLock()
        self.active_executions = {}

        # Initialize the RMF adapter.
        self.adapter = rmf_adapter.Adapter.make(
            "j100_fleet_rmf_adapter"
        )

        if self.adapter is None:
            raise RuntimeError(
                "Unable to initialize Open-RMF adapter. "
                "Verify that the RMF traffic schedule is running."
            )

        self.adapter.node.use_sim_time()

        self.adapter.start()

        self.fleet_handle = (
            self.adapter.add_easy_fleet(
                configuration
            )
        )

        if self.fleet_handle is None:
            raise RuntimeError(
                "Unable to create Open-RMF fleet"
            )

        # Passive access to Open-RMF fleet and task updates.
        #
        # This listener does not publish commands,
        # change task assignments, or control robots.

        self._rmf_update_preview_count = 0

        self.fleet_handle.more().set_update_listener(
            self._on_rmf_update
        )

        print(
            f"[RMF] Fleet created: {configuration.fleet_name}",
            flush=True,
        )

        print(
            "[RMF] Passive task update listener registered",
            flush=True,
        )

        # Create one Nav2 interface per configured robot.
        for robot_name in robot_names:

            client = Nav2RobotClient(robot_name)

            self.clients[robot_name] = client
            self.update_handles[robot_name] = None

            # Create a periodic state update timer.
            client.create_timer(
                STATE_UPDATE_PERIOD,
                lambda name=robot_name:
                    self.update_robot(name),
            )

            print(
                f"[RMF] Nav2 interface ready: {robot_name}",
                flush=True,
            )

    def _on_rmf_update(self, update):
        """Passively inspect Open-RMF task and fleet updates."""

        try:

            if isinstance(update, str):
                payload = json.loads(update)
            else:
                payload = update

            if not isinstance(payload, dict):
                print(
                    "[RMF_UPDATE_WARNING] "
                    f"Unexpected payload type: {type(payload)}",
                    flush=True,
                )
                return

            event_type = str(
                payload.get("type", "")
            )

            serialized = json.dumps(
                payload,
                separators=(",", ":"),
                ensure_ascii=False,
                default=str,
            )

            # Record every task-related update.
            # Do not continuously log fleet-state updates.

            task_related = (
                "task" in event_type.lower()
                or any(
                    key in payload
                    for key in (
                        "task_id",
                        "tasks",
                        "booking",
                    )
                )
            )

            if task_related:

                print(
                    "[RMF_TASK_UPDATE] "
                    + serialized,
                    flush=True,
                )

            elif self._rmf_update_preview_count < 5:

                # Capture a few initial examples so
                # we can verify the callback payload schema.

                self._rmf_update_preview_count += 1

                print(
                    "[RMF_UPDATE_PREVIEW] "
                    + serialized[:10000],
                    flush=True,
                )

        except Exception as error:

            # An observation failure must never
            # interrupt robot navigation.

            print(
                "[RMF_UPDATE_ERROR] "
                f"{type(error).__name__}: {error}",
                flush=True,
            )

    def create_robot_callbacks(self, robot_name):
        """Connect Open-RMF navigation requests to Nav2."""

        client = self.clients[robot_name]

        # The latest RMF navigation request waiting for Nav2.
        pending_navigation = None

        # True when cancellation of the active goal was requested.
        cancel_requested = False

        def request_replan():
            handle = self.update_handles.get(robot_name)

            if handle is not None:
                more = handle.more()

                if more is not None:
                    more.replan()

        def dispatch(destination, execution):
            """Send a reserved navigation command to Nav2."""

            nonlocal pending_navigation, cancel_requested

            x, y, yaw = (
                float(value)
                for value in destination.position
            )

            print(
                f"[RMF] Navigate {robot_name}: "
                f"({x:.3f}, {y:.3f}, {yaw:.3f})",
                flush=True,
            )

            def on_complete(succeeded, message):
                nonlocal pending_navigation, cancel_requested

                with self._execution_lock:

                    current = self.active_executions.get(
                        robot_name
                    )

                    if current is not execution:
                        return

                    replacement = pending_navigation
                    was_cancel_requested = cancel_requested

                    pending_navigation = None
                    cancel_requested = False

                    if replacement is not None:

                        # Reserve the replacement before releasing
                        # the lock, preventing another dispatch.
                        self.active_executions[robot_name] = (
                            replacement[1]
                        )

                        print(
                            f"[RMF] Starting queued navigation: "
                            f"{robot_name}",
                            flush=True,
                        )

                        dispatch(*replacement)
                        return

                    self.active_executions[robot_name] = None

                print(
                    f"[RMF] Navigation result for {robot_name}: "
                    f"success={succeeded}, {message}",
                    flush=True,
                )

                # An intentional cancellation must not trigger
                # another RMF replan.
                if was_cancel_requested:
                    return

                if succeeded:
                    execution.finished()
                else:
                    request_replan()

            try:
                client.navigate(
                    x,
                    y,
                    yaw,
                    on_complete,
                )

            except Exception as error:
                on_complete(
                    False,
                    str(error),
                )

        def navigate(destination, execution):
            nonlocal pending_navigation, cancel_requested

            if destination.map != MAP_NAME:
                print(
                    f"[RMF] Unsupported destination map for "
                    f"{robot_name}: {destination.map}",
                    flush=True,
                )
                request_replan()
                return

            with self._execution_lock:

                current = self.active_executions.get(
                    robot_name
                )

                if current is not None:

                    # Ignore duplicate requests for the same
                    # RMF activity.
                    if current.identifier.is_same(
                        execution.identifier
                    ):
                        return

                    # Replace any older pending request.
                    pending_navigation = (
                        destination,
                        execution,
                    )

                    if not cancel_requested:

                        cancel_requested = True

                        print(
                            f"[RMF] Queuing replacement and "
                            f"cancelling active navigation: "
                            f"{robot_name}",
                            flush=True,
                        )

                        client.stop()

                    return

                # Reserve this execution before dispatch.
                self.active_executions[robot_name] = execution

                dispatch(
                    destination,
                    execution,
                )

        def stop(activity):
            nonlocal pending_navigation, cancel_requested

            with self._execution_lock:

                # RMF may cancel a request that is still queued.
                if pending_navigation is not None:

                    pending_execution = pending_navigation[1]

                    if pending_execution.identifier.is_same(
                        activity
                    ):
                        pending_navigation = None

                execution = self.active_executions.get(
                    robot_name
                )

                if execution is None:
                    return

                if not execution.identifier.is_same(activity):
                    return

                if cancel_requested:
                    return

                cancel_requested = True

                print(
                    f"[RMF] Stopping navigation: {robot_name}",
                    flush=True,
                )

                # Do not clear active_executions here.
                # Wait for Nav2's terminal result.
                client.stop()

        def execute_action(category, description, execution):

            print(
                f"[RMF] Unsupported action for "
                f"{robot_name}: {category}",
                flush=True,
            )

        return rmf_easy.RobotCallbacks(
            navigate,
            stop,
            execute_action,
        )

    def make_robot_state(self, robot_name):
        """Create an RMF state from the robot's AMCL pose."""

        pose = self.clients[robot_name].get_pose()

        if pose is None:
            return None

        position = np.asarray(
            pose,
            dtype=np.float64,
        )

        return rmf_easy.RobotState(
            MAP_NAME,
            position,
            SIMULATED_BATTERY_SOC,
        )

    def register_robot(self, robot_name, state):
        """Register one localized robot with Open-RMF."""

        robot_configuration = (
            self.configuration.get_known_robot_configuration(
                robot_name
            )
        )

        callbacks = self.create_robot_callbacks(
            robot_name
        )

        handle = self.fleet_handle.add_robot(
            robot_name,
            state,
            robot_configuration,
            callbacks,
        )

        if handle is None:
            print(
                f"[RMF] Registration failed: {robot_name}",
                flush=True,
            )
            return

        self.update_handles[robot_name] = handle

        print(
            f"[RMF] Robot registration requested: "
            f"{robot_name}",
            flush=True,
        )

    def update_robot(self, robot_name):
        """Register or update a robot using its latest AMCL pose."""

        client = self.clients[robot_name]

        state = self.make_robot_state(robot_name)

        if state is None:
            return

        handle = self.update_handles[robot_name]

        if handle is None:

            # Do not register a robot until its Nav2
            # action server is available.
            if not client.navigation_ready():
                return

            self.register_robot(
                robot_name,
                state,
            )

            return

        with self._execution_lock:

            execution = self.active_executions.get(
                robot_name
            )

            current_activity = (
                execution.identifier
                if execution is not None
                else None
            )

        handle.update(
            state,
            current_activity,
        )


def main():
    parser = argparse.ArgumentParser(
        description="Open-RMF J100 fleet adapter"
    )

    parser.add_argument(
        "--fleet",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--rmf-config",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--graph",
        type=Path,
        required=True,
    )

    args = parser.parse_args()

    configuration, robot_names = build_rmf_configuration(
        args.fleet,
        args.rmf_config,
        args.graph,
    )

    print("===== RMF FLEET ADAPTER =====", flush=True)
    print(
        f"Fleet: {configuration.fleet_name}",
        flush=True,
    )
    print(
        f"Robots: {robot_names}",
        flush=True,
    )

    rclpy.init()

    rmf_adapter.init_rclcpp()

    bridge = None

    executor = rclpy.executors.SingleThreadedExecutor()

    try:

        bridge = RMFFleetBridge(
            configuration,
            robot_names,
        )

        for client in bridge.clients.values():
            executor.add_node(client)

        print(
            "[RMF] Waiting for localized robots "
            "and active Nav2 servers...",
            flush=True,
        )

        executor.spin()

    except KeyboardInterrupt:
        print(
            "[RMF] Shutdown requested",
            flush=True,
        )

    finally:

        executor.shutdown()

        if bridge is not None:
            for client in bridge.clients.values():
                client.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
# AMR Simulation Scalability and Control Reliability

An experimental investigation into the scalability, performance, and control reliability of multi-robot simulations using **ROS 2 Jazzy, Gazebo Harmonic, Nav2, and Clearpath Robotics**.

The project investigates how simulation performance and robot responsiveness change as the number of simulated autonomous mobile robots (AMRs) increases.

The objective is to identify performance bottlenecks through controlled, reproducible experiments rather than assuming that simulation slowdown and loss of robot control have the same underlying cause.

## Current Validation Status

**Last validated:** November 20, 2026

The project has completed its initial single-robot experiments and established a validated two-robot simulation and autonomous navigation baseline.

### Implementation Progress

| Component | Status |
|---|---|
| Single-robot simulation and control | Validated |
| Single-robot autonomous navigation | Validated |
| Multi-robot simulation | Validated with 2 robots |
| Independent ROS 2 namespaces | Validated |
| Independent AMCL localization | Validated |
| Independent Nav2 navigation stacks | Validated |
| Controlled Nav2 lifecycle activation | Validated |
| Simultaneous two-robot navigation | Validated |
| Two-robot repeatability experiments | Completed |
| Open-RMF fleet management | Pending integration |
| Automated end-to-end fleet initialization | Pending |
| Scaling to 3, 5, and 10 robots | Planned |

### Two-Robot Navigation Baseline

Three independent two-robot navigation experiments were completed using identical configurations, initial positions, and navigation goals.

| Metric | Result |
|---|---|
| Independent experiments | 3 |
| Navigation actions | 6 |
| Successful navigation actions | 6/6 |
| Mean fleet makespan | 12.468 seconds |
| Recovery behaviors | 0 |
| Controlled Nav2 startup attempts | 6/6 successful |

All navigation actions completed successfully.

Independent Gazebo ground-truth positioning measurements were also collected and preserved. Four of six final positions satisfied the additional 0.25 m positioning-accuracy criterion. This criterion is distinct from successful Nav2 action completion.

The current implementation demonstrates simultaneous autonomous navigation without centralized fleet coordination.

**Detailed documentation:** [Two-Robot Simulation and Navigation Baseline](docs/baseline-2-robots.md).

### Next Development Milestone: Open-RMF

The next phase will integrate Open-RMF into the existing Docker-based architecture.

The objective is to establish a reproducible two-robot fleet-management baseline with:

- Open-RMF fleet registration and robot-state reporting.
- A warehouse navigation and traffic graph.
- A fleet adapter connecting Open-RMF to the existing Nav2 interfaces.
- RMF-managed navigation tasks and traffic coordination.
- Automated startup, initialization, execution, and result collection.
- Repeatable benchmarks for comparing direct Nav2 navigation against Open-RMF-managed operation.

The validated Nav2-only baseline will remain unchanged as a reference.

Fleet scaling to 3, 5, and 10 robots will follow successful Open-RMF integration and validation.

## 1. Research Objectives

This investigation examines how increasing the number of simulated AMRs affects:

1. Simulation performance and real-time execution.
2. Robot-command responsiveness and control reliability.
3. Computational resource utilization.
4. Sensor data publication and communication.
5. Overall simulation stability.
6. Autonomous navigation performance and reliability.
7. Localization and positioning accuracy.
8. Multi-robot coordination and mission execution.

The experiments aim to distinguish between potential bottlenecks in physics computation, sensor rendering, robot controllers, localization, autonomous navigation, fleet coordination, and ROS 2 communication.

## 2. Simulation Environment

### Hardware and Software

| Component | Configuration |
|---|---|
| Host OS | Ubuntu 24.04 LTS |
| CPU | AMD Ryzen 5 7600 |
| CPU Cores / Threads | 6 / 12 |
| System Memory | 32 GB DDR5 |
| Containerization | Docker and Docker Compose |
| ROS Distribution | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic 8.15.0 |
| Robot Platform | Clearpath Jackal J100 |
| Simulation World | Clearpath Warehouse |
| GPU | NVIDIA GeForce RTX 3060 (12 GB) |
| Robot Sensors | Hokuyo 2D LiDAR and simulated IMU |
| Controller Update Rate | 50 Hz (configured) |
| ROS Middleware | Fast DDS (`rmw_fastrtps_cpp`) |
| ROS Domain ID | 0 |
| Navigation Framework | Nav2 |
| Global Planner | NavFn |
| Local Controller | MPPI |
| Localization | AMCL |
| Simulation Architecture | Single Gazebo server |
| Initial Robot Count | 1 |

### Container Environment

The simulation runs inside Docker to minimize host dependencies and improve reproducibility.

The initial environment uses Clearpath's official simulation image:

`ghcr.io/clearpathrobotics/clearpath_docker:jazzy-sim-latest`

For reproducibility, the Docker Compose configuration references the following image digest:

```text
sha256:970015d533d52c816d1d59ec10cfc2d5df20977b8d8d34c89c67f58ee6543315
```

The repository also includes the upstream Clearpath Simulator as a Git submodule, pinned to commit:

```text
7a3e5d38dcc37b83dab1de0d986501234d669f78
```

**Important:** The current Docker environment uses Clearpath packages installed in the official container image. The Git submodule provides a version-controlled upstream source reference but is not currently compiled into the container. Package versions inside the image must be documented independently.

### Modular Container Architecture

The current architecture separates simulation, visualization, localization, and autonomous navigation into independent Docker services.

| Service | Container | Responsibility |
|---|---|---|
| `simulation` | `amr-scalability-simulation` | Gazebo, robot physics, sensors, controllers, and ROS interfaces |
| `visualization` | `amr-scalability-visualization` | Optional RViz2 visualization |
| `localization` | `amr-scalability-localization` | Static map server and AMCL |
| `navigation` | `amr-scalability-navigation` | Nav2 planning, control, behavior trees, and recovery behaviors |

All services use host networking and host IPC to support same-host ROS 2 communication.

Simulation and visualization use the pinned Clearpath image.

Localization and navigation use a custom image derived from the same pinned Clearpath image:

```text
amr-scalability-nav2:jazzy
```

The custom image is built from:

```text
docker/nav2/Dockerfile
```

The Docker Compose configuration is divided into three files:

```text
compose.yaml
compose.visualization.yaml
compose.navigation.yaml
```

This separation allows components to be started and stopped independently. It also enables independent measurement of simulation, localization, and navigation resource consumption.

RViz2 is disabled during standard performance experiments unless visualization overhead is an explicit experimental variable.

### Robot Configuration

The initial robot is a Clearpath Jackal J100 configured with:

- Four-wheel differential/skid-steer motion control.
- Wheel odometry.
- A front-mounted Hokuyo 2D LiDAR.
- Simulated inertial measurements.
- ROS 2 velocity-command interfaces.

Additional rendering-intensive sensors, such as 3D LiDAR and cameras, may be introduced as controlled experimental variables.

The source robot configuration is stored in:

```text
config/jackal/robot.yaml
```

Generated controller, robot-description, and sensor-configuration files are excluded from version control because Clearpath recreates them during initialization.

### Navigation Configuration

The navigation configuration is stored in:

```text
config/navigation/j100/nav2.yaml
```

The file is based on the Clearpath Nav2 configuration for the Jackal J100, with corrections to the local and global costmap footprints.

It is mounted into the navigation container at the location expected by the upstream Clearpath navigation launch file.

The current navigation architecture uses the following TF chain:

```text
map
 |
 | AMCL
 v
odom
 |
 | Robot localization
 v
base_link
 |
 v
Robot links and sensors
```

The initial robot namespace is:

```text
j100_0000
```

TF topics, robot sensor topics, velocity commands, and Nav2 actions are namespaced.

## 3. Experimental Methodology

The investigation proceeds incrementally.

### Phase 1 — Environment and Functional Validation

Establish a working simulation containing one Jackal.

Validation includes:

- Launching the warehouse environment.
- Spawning the robot.
- Activating its controllers.
- Verifying velocity-command execution.
- Confirming odometry publication.
- Confirming sensor-data publication.
- Establishing reliable performance-measurement procedures.

**Status:** Functional validation and automated performance measurement validation completed. Independent warehouse physics collision verification remains pending.

### Phase 2 — Single-Robot Performance Baseline

Develop automated experiments to establish reference measurements for one sensor-equipped Jackal.

The baseline measures:

| Metric | Purpose |
|---|---|
| Real-Time Factor (RTF) | Simulation speed relative to real time |
| CPU Utilization | Processor workload |
| Memory Utilization | Resource consumption |
| GPU Utilization | Rendering and computational workload |
| LiDAR Publication Rate | Sensor performance |
| Command Responsiveness | Robot control behavior |
| Odometry | Observed simulated motion |
| Tracking Error | Difference between commanded and observed behavior |
| Planning Time | Navigation planning performance |
| Navigation Duration | Autonomous mission completion time |
| Trajectory Length | Executed motion relative to the initial plan |
| Goal Position Error | Final positioning accuracy |
| Recovery Frequency | Navigation reliability |
| Action Feedback Rate | Navigation communication and monitoring workload |
| Controller Timing Warnings | Potential navigation scheduling problems |

Measurements distinguish wall-clock time from simulation time.

Idle simulation, DDS communication, motion control, and autonomous navigation are evaluated separately to avoid conflating different workloads.

**Status:** Completed for one simulated Jackal. Recorded limitations and anomalies are retained as baseline findings.

### Phase 3 — Incremental Multi-Robot Scaling

Increase the number of simulated Jackals while maintaining a consistent robot configuration and experimental workload.

The initial planned robot counts are:

```text
1 → 2 → 3 → 5 → 10 → 20 → ... → 100+
```

The progression will be adjusted according to measured results and hardware limitations.

Each robot will have:

- A unique identity and namespace.
- An independent control interface.
- An identical baseline sensor configuration.
- A defined starting position.
- A reproducible control workload.
- Independent localization and navigation interfaces.

Experiments will use repeated runs under controlled conditions.

The two-robot Nav2-only baseline was validated on November 20, 2026. Open-RMF fleet coordination will be integrated and benchmarked before scaling beyond two robots.

### Phase 4 — Bottleneck Investigation

When performance degradation or control failures are observed, investigate potential causes individually.

Areas of investigation include:

- Physics computation.
- Sensor simulation and rendering.
- ROS 2 communication and discovery.
- Controller execution.
- Localization and navigation computation.
- CPU and GPU resource saturation.
- Simulation scheduling and timing.
- Fleet coordination and traffic management.

Controlled comparisons will be used to distinguish correlation from supported explanations.

For example, disabling LiDAR while preserving robot count and control workload can help estimate the performance impact of sensor simulation.

Existing single-robot navigation recoveries and localization discrepancies must be considered when interpreting multi-robot experimental results.

### Phase 5 — Analysis and Conclusions

Analyze the experimental measurements to determine:

- How simulation performance scales with robot count.
- Whether robot responsiveness changes as simulation load increases.
- Which resources become limiting.
- Whether failures originate in simulation, communication, localization, navigation, or control.
- How navigation success, accuracy, and recovery frequency change with robot count.
- Which changes measurably improve performance or reliability.

Conclusions will be supported by recorded measurements and documented experimental conditions.

## 4. Reproducing the Simulation

### Prerequisites

The current setup requires:

- Ubuntu or another compatible Linux environment.
- Docker Engine and Docker Compose.
- NVIDIA container support for the current GPU-enabled configuration.
- A graphical environment compatible with the Docker Compose display configuration.
- Python 3 on the host for experimental collection and analysis scripts.

ROS 2 and Gazebo do not need to be installed on the host.

### Clone the Repository

```bash
git clone --recurse-submodules \
  https://github.com/mathrosas/amr-simulation-scalability.git

cd amr-simulation-scalability
```

### Launch the Simulation

```bash
docker compose up
```

This launches the Clearpath warehouse environment with one configured Jackal J100.

For the current configuration, Gazebo's graphical interface opens on the host.

### Verify Container Health

In another terminal:

```bash
docker ps
```

The simulation container should report a healthy status after initialization.

### Access the ROS 2 Environment

```bash
docker compose exec simulation bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
```

ROS 2 commands can then be executed without installing ROS on the host.

### Independent RViz2 Visualization

Start the visualization service while the simulation is running:

```bash
docker compose \
  -f compose.yaml \
  -f compose.visualization.yaml \
  up -d --no-deps visualization
```

Stop visualization independently:

```bash
docker compose \
  -f compose.yaml \
  -f compose.visualization.yaml \
  stop visualization
```

The default visualization configuration is:

```text
config/rviz/config.rviz
```

An alternative configuration for autonomous navigation is available at:

```text
config/rviz/navigation.rviz
```

### Build the Navigation Image

Define a Bash array containing all three Compose files:

```bash
DC=(
  docker compose
  -f compose.yaml
  -f compose.visualization.yaml
  -f compose.navigation.yaml
)
```

Build the custom image:

```bash
"${DC[@]}" build navigation
```

The localization service uses the same custom image.

The base Clearpath image is digest-pinned. Additional ROS packages installed by the Dockerfile are not yet pinned to exact package versions, so rebuilding the image at a later date may produce different software versions.

### Start Simulation and Localization

Start the simulation:

```bash
"${DC[@]}" up -d --no-deps --no-build simulation
```

Wait for Gazebo to finish initialization.

Verify the physical starting pose:

```bash
"${DC[@]}" exec simulation bash -lc '
source /opt/ros/jazzy/setup.bash

gz model -m "j100_0000/robot" --pose
'
```

For the current single-robot configuration, the expected initial position is:

```text
XYZ: (0.0, 0.0, 0.0635)
RPY: (0.0, 0.0, 0.0)
```

Start localization:

```bash
"${DC[@]}" up -d --no-deps --no-build localization
```

Inspect the localization logs:

```bash
docker logs --tail 25 amr-scalability-localization
```

Wait until the localization lifecycle manager reports that its managed nodes are active.

### Initialize AMCL

AMCL requires an initial position estimate before it can establish the `map → odom` transformation.

The initial estimate must correspond to the robot's actual starting position.

For the current single-robot spawn configuration:

```bash
"${DC[@]}" exec localization bash -lc '
source /opt/ros/jazzy/setup.bash

timeout 20s ros2 topic pub --once -w 1 \
  /j100_0000/initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{
    header: {
      frame_id: map,
      stamp: {sec: 0, nanosec: 0}
    },
    pose: {
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      covariance: [
        0.25, 0, 0, 0, 0, 0,
        0, 0.25, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0.06854
      ]
    }
  }"
'
```

This initialization procedure will be generalized in later multi-robot development stages, where each robot will have its own initial position.

Do not assign the origin as the initial pose of a robot that has already moved away from the origin.

### Verify Localization

```bash
"${DC[@]}" exec localization bash -lc '
source /opt/ros/jazzy/setup.bash

timeout 12s ros2 run tf2_ros tf2_echo map base_link \
  --ros-args \
  -r /tf:=/j100_0000/tf \
  -r /tf_static:=/j100_0000/tf_static \
  -p use_sim_time:=true
'
```

Initial transform-discovery warnings may occur. Wait for valid and consistent transformations before continuing.

### Start Autonomous Navigation

```bash
"${DC[@]}" up -d --no-deps --no-build navigation
```

Inspect the navigation logs:

```bash
docker logs --tail 25 amr-scalability-navigation
```

Wait for:

```text
Managed nodes are active
```

Verify the main navigation lifecycle states:

```bash
"${DC[@]}" exec navigation bash -lc '
source /opt/ros/jazzy/setup.bash

for node in controller_server planner_server bt_navigator; do
    echo
    echo "===== $node ====="

    timeout 15s ros2 service call \
      "/j100_0000/$node/get_state" \
      lifecycle_msgs/srv/GetState "{}"
done
'
```

Each node should report lifecycle state `active` with ID `3`.

The navigation probe additionally verifies simulation clock progression, fresh localization and odometry data, and action server availability before executing a goal.

### Visualize Autonomous Navigation

Optionally launch RViz2 using the navigation configuration:

```bash
RVIZ_CONFIG_FILE=./config/rviz/navigation.rviz \
  "${DC[@]}" up -d \
  --no-deps \
  --no-build \
  --force-recreate visualization
```

The navigation configuration includes:

- Static warehouse map.
- Robot model.
- Namespaced TF transformations.
- LiDAR scan.
- Global costmap.
- Local costmap.
- Global navigation path.
- Published robot footprint.

The RViz2 configuration is mounted read-only. To persist changes, save the configuration to a writable path inside the container and copy the updated file back to the repository.

### Execute a Controlled Navigation Experiment

Stop RViz2 before collecting standard performance measurements:

```bash
"${DC[@]}" stop -t 10 visualization
```

The trial runner requires simulation, localization, and navigation to be running, with the robot at the original spawn position.

Execute one controlled trial:

```bash
REPEAT_DIR="results/raw/navigation_repeat_1robot_$(date +%Y%m%d_%H%M%S)"

python3 scripts/run_navigation_trial.py \
  --output-dir "$REPEAT_DIR/trial_01" \
  --goal-x -1.2874 \
  --goal-y -3.1625 \
  --goal-yaw 0.0 \
  --interval 2.0
```

Analyze the experiment:

```bash
python3 scripts/analyze_navigation_probe.py \
  "$REPEAT_DIR/trial_01" \
  --assume-map-world-aligned
```

The alignment flag explicitly permits direct comparison of map-frame localization estimates with Gazebo world-frame ground-truth positions. These comparisons remain conditional on the alignment assumption.

For repeatability experiments, restart simulation, localization, and navigation before each repetition and reinitialize AMCL at the verified starting pose.

Use separate output directories for each trial.

Aggregate three completed navigation trials:

```bash
python3 scripts/analyze_navigation_repeat.py \
  "$REPEAT_DIR/trial_01" \
  "$REPEAT_DIR/trial_02" \
  "$REPEAT_DIR/trial_03" \
  --output "$REPEAT_DIR/aggregate.json"
```

Aggregate resource consumption and verify recorded experimental configuration consistency:

```bash
python3 scripts/analyze_navigation_resources.py \
  "$REPEAT_DIR/trial_01" \
  "$REPEAT_DIR/trial_02" \
  "$REPEAT_DIR/trial_03" \
  --output "$REPEAT_DIR/resource_aggregate.json"
```

The trial runner currently assumes the single-robot namespace, warehouse world, and original spawn position. These assumptions will be generalized for subsequent fleet experiments.

If navigation becomes unexpected or action cancellation cannot be confirmed, stop the navigation service from a separate terminal and verify that the robot has stopped.

### Stop the Simulation

Stop navigation and localization before stopping simulation:

```bash
"${DC[@]}" stop -t 10 navigation localization
```

Stop visualization and simulation without removing their containers:

```bash
"${DC[@]}" stop -t 45 visualization simulation
```

Inspect all service states:

```bash
"${DC[@]}" ps -a
```

Simulation and visualization have sometimes required forced termination after the configured shutdown timeout, producing exit code `137`. This remains a known infrastructure limitation.

The Gazebo model cache is stored in a persistent Docker volume.

The visualization configuration and experimental results are stored in the repository independently of the running containers.

## 5. Experimental Results

### Milestone 0 — Repository Initialization

**Status: Completed**

The research repository was initialized and connected to GitHub.

The official Clearpath Simulator was included as a Git submodule.

The initial research objectives, experimental methodology, and project structure were established.

### Milestone 1 — Container Environment Configuration

**Status: Completed**

The official Clearpath Docker simulation image was downloaded and validated.

The following software components were confirmed:

| Component | Result |
|---|---|
| ROS 2 | Jazzy |
| Gazebo | Harmonic 8.15.0 |
| Clearpath Gazebo Package | Installed |
| NVIDIA Container Runtime | Available |
| Jackal Configuration Parser | Validation passed |

A sensor-equipped Jackal J100 configuration was created and validated using Clearpath's configuration parser.

### Milestone 2 — Single-Robot Functional Validation

**Date:** November 15, 2026

**Status:** Functional validation passed.

#### Objective

Verify that a simulated Jackal can be spawned, controlled, and observed through ROS 2 while publishing sensor data.

#### Environment

- Gazebo Harmonic 8.15.0.
- ROS 2 Jazzy.
- Clearpath warehouse world.
- One Jackal J100.
- One front-mounted Hokuyo 2D LiDAR.
- Simulated IMU.
- Active wheel controllers.

#### Controller Verification

Both controllers were confirmed active through the ROS 2 controller manager.

| Controller | State | Configured Update Rate |
|---|---|---|
| `joint_state_broadcaster` | Active | 50 Hz |
| `platform_velocity_controller` | Active | 50 Hz |

These values represent the controllers' configured update rates, not independently measured execution frequencies.

#### Sensor Verification

The Gazebo simulator successfully initialized the 2D LiDAR.

The corresponding ROS 2 bridge was created, and a `sensor_msgs/msg/LaserScan` message was successfully received.

Filtered odometry was also received through ROS 2.

The IMU initialized and reported receiving sensor data.

#### Movement Test

A sequence of velocity commands was published through ROS 2.

| Parameter | Value |
|---|---|
| Commanded Linear Velocity | 0.2 m/s |
| Commanded Angular Velocity | 0 rad/s |
| Command Publication Rate | 10 Hz |
| Number of Commands | 30 |
| Initial X Position | Approximately 0 m |
| Final X Position | Approximately 0.718 m |
| Reported X Displacement | Approximately 0.718 m |

The robot was visually observed moving forward and stopping correctly after the command sequence.

The nominal commanded travel distance for three seconds at 0.2 m/s is 0.6 m.

However, this manual test did not synchronize command timestamps, simulation time, and odometry measurements. Therefore, the difference between the nominal distance and the reported displacement cannot yet be interpreted as a measured tracking error.

#### Outstanding Observations

**1. ROS 2 discovery inconsistencies**

ROS 2 topic-listing commands returned incomplete or inconsistent information.

Direct subscriptions using explicit message types successfully received odometry and LiDAR data.

The underlying cause remains undetermined.

**2. Physics collision warnings**

Gazebo reported DART physics warnings indicating that certain mesh collision geometries could not be created.

Warehouse obstacle collisions require independent verification before physical collision behavior can be evaluated.

**3. Simulation timing**

The movement test confirmed functional command execution but did not measure controller latency or trajectory-tracking accuracy.

These measurements require an automated test using synchronized timestamps.

**4. Asset caching**

The initial warehouse launch downloaded Gazebo Fuel resources.

Subsequent benchmark experiments must account for differences between initial startup and runs using cached assets.

#### Conclusion

The single-Jackal functional validation was successful.

The simulation demonstrated:

- Successful robot spawning.
- Controller initialization.
- ROS 2 velocity-command execution.
- Forward movement and automatic stopping.
- Odometry publication.
- LiDAR data publication.

This establishes a functional foundation for subsequent benchmarking.

**No conclusions about multi-robot scalability, command latency, or performance degradation can yet be drawn from this initial experiment.**

### Milestone 3 — Automated Single-Robot Baseline

**Dates:** November 16–20, 2026

**Status:** Completed.

#### Simulation Performance and Resource Monitoring

Automated collection tools were developed to measure Gazebo real-time factor, CPU utilization, memory consumption, and GPU utilization.

The unified baseline runner combines simulation statistics, resource measurements, and optional ROS 2 sensor timing measurements.

Relevant scripts:

- `scripts/collect_rtf.py`
- `scripts/collect_resources.py`
- `scripts/run_baseline.py`

#### ROS 2 Sensor Timing

Automated monitoring was implemented for:

- LiDAR.
- Raw odometry.
- Filtered odometry.
- IMU.
- Joint states.

Measurements include message arrival timestamps, ROS message header timestamps, arrival intervals, and message timestamp intervals.

Relevant scripts:

- `scripts/ros_sensor_monitor.py`
- `scripts/analyze_sensor_timing.py`

#### DDS Communication Benchmark

A controlled same-host ROS 2 publisher/subscriber benchmark was implemented to measure application-level communication performance.

The benchmark records latency, message delivery, missing messages, duplicate messages, and message ordering.

Three repeatability trials were conducted using Reliable QoS.

The DDS benchmark is independent of robot motion and should not be interpreted as a measurement of complete robot-control latency.

Relevant scripts:

- `scripts/dds_probe.py`
- `scripts/analyze_dds_probe.py`

#### Automated Motion-Control Benchmark

A motion-control probe was developed to evaluate the response of the Clearpath Jackal to repeatable velocity commands.

Four observation points are included in the final implementation:

| Stream | Observation point |
|---|---|
| Requested command | Local velocity publication |
| Multiplexer output | `/j100_0000/platform/cmd_vel` |
| Controller output | `/j100_0000/platform_velocity_controller/cmd_vel_out` |
| Odometry | `/j100_0000/platform/odom` |

The probe records monotonic wall-clock timestamps, simulation time, ROS message header timestamps, velocity, and odometry position.

The controller was verified to use feedback-based odometry (`open_loop=false`).

##### Experimental Configuration

| Parameter | Value |
|---|---|
| Robot count | 1 |
| Linear velocity | 0.10 m/s |
| Angular velocity | 0 rad/s |
| Command publication frequency | 20 Hz |
| Initial stationary period | 1 s |
| Movement duration | 2 s (wall time) |
| Stopping observation period | 2 s |
| Controller command timeout | 0.5 s |
| Gazebo GUI | Enabled |
| RViz2 | Disabled during repeatability trials |
| Nav2 | Not running |
| Open-RMF | Not running |

One functional smoke test and two sets of three repeatability trials were conducted.

The initial smoke test was conducted with RViz2 running. The repeatability datasets were collected with RViz2 stopped.

The second repeatability dataset included multiplexer instrumentation.

##### Instrumented Repeatability Results

Results from three trials:

| Metric | Mean | Sample standard deviation |
|---|---:|---:|
| Command frequency (Hz) | 19.9541 | 0.0050 |
| Controller 90% response (ms) | 49.582 | 0.085 |
| Odometry 90% response (ms) | 217.704 | 0.167 |
| Multiplexer stopping response (ms) | 0.179 | 0.008 |
| Controller stopping response (ms) | 23.415 | 0.230 |
| Odometry stopping response (ms) | 212.552 | 0.209 |
| Steady-state odometry velocity (m/s) | 0.100060 | 0.000105 |
| Odometry displacement (mm) | 198.240 | 0.073 |
| Displacement error (%) | -0.930 | 0.036 |
| Effective motion RTF | 1.000441 | 0.000004 |

All three instrumented trials completed successfully and reached effectively zero odometry velocity during the final observation window.

##### Stopping Anomaly

One trial in the original repeatability dataset exhibited an unexpected stopping delay.

The controller continued reporting approximately 0.10 m/s for approximately 250 ms after zero-velocity commands began.

The observed odometry stopping response was 465.116 ms, compared with approximately 212.5 ms in the other two original trials.

Odometry displacement increased from approximately 198.3 mm to 223.5 mm.

Inspection of the recorded velocity profiles indicated that the additional delay occurred before the controller output decreased, rather than during the subsequent odometry deceleration.

Multiplexer monitoring was subsequently added to improve visibility into the command pipeline.

The anomaly did not recur in the three instrumented trials.

Its underlying cause remains undetermined. All experimental results, including the anomalous trial, are retained.

##### Experimental Limitations

- Odometry is not independent Gazebo ground truth.
- Observed response times include communication, processing, controller updates, simulated dynamics, and callback scheduling.
- The experiments do not measure pure DDS latency through the complete robot-control pipeline.
- Displacement includes the deceleration period.
- Starting poses were not reset between consecutive trials.
- Three trials per configuration provide preliminary repeatability measurements, not statistically established reliability limits.
- These results do not establish multi-robot scalability.

##### Scripts and Datasets

Relevant scripts:

- `scripts/control_probe.py`
- `scripts/analyze_control_probe.py`
- `scripts/analyze_control_repeat.py`

Experimental datasets:

- `results/raw/control_probe_smoke_1robot_20261118_061410/`
- `results/raw/control_probe_repeat_1robot_20261118_061938/`
- `results/raw/control_probe_mux_repeat_1robot_20261118_064334/`

Individual experiments contain raw CSV measurements and JSON analysis.

Repeated experiments also include aggregate descriptive statistics.

#### Independent RViz2 Visualization

RViz2 was validated using the Clearpath visualization configuration.

The following functionality was verified:

- Jackal robot model visualization.
- Namespaced TF transformations.
- Simulation clock synchronization.
- LiDAR visualization.
- Simultaneous Gazebo and RViz2 operation.
- Independent visualization service startup and shutdown.

RViz2 runs in a separate Docker container using the same pinned Clearpath image as the simulation.

Configuration files:

- `compose.visualization.yaml`
- `config/rviz/config.rviz`
- `config/rviz/navigation.rviz`

Visualization is disabled during standard performance experiments unless its computational overhead is an experimental variable.

### Milestone 3G — Autonomous Navigation and Navigation Benchmarking

**Dates:** November 18–20, 2026

**Status:** Completed for one simulated Jackal.

#### Objective

Integrate autonomous navigation into the existing single-robot simulation and establish a reproducible reference workload for future multi-robot navigation and fleet coordination experiments.

The navigation experiments evaluate:

- Localization using a predefined occupancy-grid map.
- Obstacle-aware global path planning.
- Autonomous goal execution.
- Navigation action responsiveness.
- Trajectory length and final positioning accuracy.
- Navigation recovery behavior.
- Independent simulation, localization, and navigation resource consumption.

#### Nav2 Integration

A custom navigation image was developed using the pinned Clearpath simulation image as its base.

The custom image installs Nav2, Nav2 Bringup, Clearpath Nav2 Demos, and explicitly selected dependencies required to maintain compatibility with the Nav2 packages inherited from the base image.

The following principal package versions were used during the controlled navigation experiments:

| Package | Version |
|---|---|
| Nav2 | 1.3.14 |
| Clearpath Nav2 Demos | 2.9.0 |
| BehaviorTree.CPP | 4.11.0 |
| SLAM Toolbox | 2.8.5 |

During the initial integration, the inherited Clearpath image contained a mixture of Nav2 package versions.

The first navigation startup attempts failed during behavior-tree navigator configuration. Updating the inherited Nav2 packages exposed an unresolved symbol in a Nav2 behavior-tree plugin.

The missing symbol was:

```text
BT::formatForbiddenChar[abi:cxx11](char)
```

The installed BehaviorTree.CPP version was 4.10.0, while the updated navigation packages required a symbol provided by BehaviorTree.CPP 4.11.0.

The Dockerfile was updated to install the compatible BehaviorTree.CPP package and upgrade the relevant inherited Nav2 packages explicitly.

The required symbol was subsequently verified in the shared library, and the affected behavior-tree plugin no longer reported unresolved symbols during dependency inspection.

Following these changes, all managed navigation nodes activated successfully.

The corrected navigation image was used for both navigation and localization.

Recorded image identifier:

```text
sha256:c53cc7cfce6c2a95cee9b03c1070f6039c6192c477abf1756542544ced1d52fb
```

The base simulation and visualization images remained unchanged.

#### Localization and Occupancy Grid

Localization uses the warehouse occupancy-grid map supplied by Clearpath Nav2 Demos.

| Parameter | Value |
|---|---|
| Map width | 604 cells |
| Map height | 1004 cells |
| Resolution | 0.05 m/cell |
| Physical width | 30.2 m |
| Physical height | 50.2 m |
| Map origin X | -15.1 m |
| Map origin Y | -25.0 m |
| Map origin yaw | 0 rad |
| Occupied threshold | 0.65 |
| Free threshold | 0.10 |
| Localization algorithm | AMCL |
| Motion model | Differential |
| Minimum particles | 500 |
| Maximum particles | 2000 |
| Global reference frame | `map` |
| Odometry reference frame | `odom` |
| Robot base frame | `base_link` |

The map contains 606,416 cells.

AMCL initialization was performed by publishing an explicit initial pose corresponding to the robot's verified physical spawn position.

The initial localized position was approximately:

```text
X: -0.015 m
Y: -0.035 m
Yaw: -0.244 degrees
```

The physical robot was spawned at the origin with zero yaw.

The small initial localization discrepancy was consistent across simulation restarts.

The localization startup sequence must be completed before autonomous navigation begins.

#### Navigation Configuration and Velocity Pipeline

The following navigation settings were retained during controlled experiments:

| Parameter | Value |
|---|---|
| Global planner | NavFn |
| Planner identifier | `GridBased` |
| Local controller | MPPI |
| Controller identifier | `FollowPath` |
| Configured controller frequency | 20 Hz |
| Maximum configured forward velocity | 0.5 m/s |
| Goal position tolerance | 0.25 m |
| Goal yaw tolerance | 0.25 rad |
| Progress checker movement radius | 0.5 m |
| Progress checker time allowance | 10 s |
| Global costmap frame | `map` |
| Local costmap frame | `odom` |
| Local costmap dimensions | 5 x 5 m |
| Costmap resolution | 0.05 m |
| Global costmap inflation radius | 0.8 m |
| Velocity message type | `geometry_msgs/msg/TwistStamped` |

The navigation velocity pipeline was verified as:

```text
Nav2 controller_server
        |
        v
cmd_vel_nav
        |
        v
velocity_smoother
        |
        v
cmd_vel_smoothed
        |
        v
collision_monitor
        |
        v
cmd_vel
        |
        v
Clearpath twist_mux
        |
        v
platform/cmd_vel
        |
        v
platform_velocity_controller
        |
        v
Robot motion
```

All velocity topics are within the robot namespace.

The existing Clearpath controller and command multiplexer configurations were preserved.

#### Robot Footprint Correction

The original Clearpath navigation configuration defined a rectangular footprint measuring 0.36 x 0.30 m.

Inspection of the generated robot description showed that this footprint did not cover the complete chassis and wheel collision geometry.

| Geometry | Dimensions |
|---|---|
| Chassis collision box | 0.42 x 0.31 m |
| Chassis and wheel planar collision envelope | Approximately 0.458 x 0.41559 m |
| Original Nav2 footprint | 0.36 x 0.30 m |
| Corrected Nav2 footprint | 0.48 x 0.44 m |
| Configured footprint padding | Approximately 0.01 m |

The corrected footprint provides a conservative rectangular approximation of the main robot collision geometry.

The following polygon is used by both costmaps:

```yaml
footprint: "[ [0.24, 0.22], [0.24, -0.22], [-0.24, -0.22], [-0.24, 0.22] ]"
```

The project-specific navigation configuration replaces the corresponding upstream configuration file through a read-only Docker bind mount.

The corrected values were verified through the active local and global costmap parameters.

No progress checker, goal tolerance, controller velocity, or planner tuning was performed after this footprint correction.

#### Manual Planning and Navigation Validation

Initial functional validation was performed using a short navigation goal approximately one meter from the robot's starting position.

The planning-only action generated a valid path, and the subsequent navigation action completed successfully.

A second validation route was selected to require navigation around a warehouse shelving obstacle.

Goal:

```text
Frame: map
X: -1.2874 m
Y: -3.1625 m
Yaw: 0.0 rad
```

The global planner produced an obstacle-avoiding path that was visually verified in RViz2 against the warehouse map and Gazebo environment.

| Metric | Obstacle Planning Test |
|---|---:|
| Path poses | 242 |
| Direct distance | 3.3682 m |
| Planned path length | 6.0615 m |
| Path length ratio | 1.7996 |
| Additional distance relative to direct path | Approximately 2.6933 m |

The first manual execution of this route completed successfully in approximately 65.695 seconds.

One progress checker failure and one local costmap clearing event occurred before successful completion.

Visual obstacle-aware navigation was validated. Independent verification of the underlying warehouse physics collision geometries remains outstanding.

#### Automated Navigation Instrumentation

A navigation probe was implemented using ROS 2 action clients for:

```text
nav2_msgs/action/ComputePathToPose
nav2_msgs/action/NavigateToPose
```

The probe supports planning-only and autonomous execution modes.

Execution must be enabled explicitly.

The probe records:

- Monotonic wall-clock timestamps.
- Simulation timestamps.
- Action acceptance latency.
- Planning result and path geometry.
- Navigation result and execution duration.
- Action feedback and recovery counts.
- Map-frame localization estimates.
- Raw odometry position and orientation.
- Raw odometry linear and angular velocity.
- Estimated final goal error.
- Final stationary state.

The telemetry sampling target is 10 Hz.

Raw odometry messages are also processed separately to estimate trajectory length at the received odometry frequency.

Following normal navigation action completion, the probe includes a two-second settling observation period.

The probe attempts action cancellation when execution fails or is interrupted. If cancellation cannot be confirmed, the result indicates that manual intervention may be required.

#### Automated Smoke Tests

A planning-only smoke test generated a return path from the opposite side of the warehouse shelving obstacle to the original spawn position.

The subsequent automated execution test followed the return route.

| Metric | Planning-Only Smoke Test | Execution Smoke Test |
|---|---:|---:|
| Planning status | SUCCEEDED | SUCCEEDED |
| Navigation status | Not executed | SUCCEEDED |
| Planned path length | 5.8744 m | 5.8744 m |
| Path poses | 232 | 232 |
| Nav2 planning time | 3 ms | 3 ms |
| Navigation duration | Not applicable | 61.385 s |
| Odometry trajectory length | Not applicable | 6.1417 m |
| Recoveries | Not applicable | 1 |
| Final stationary state | Yes | Yes |

Both smoke tests completed successfully.

The automated execution test reproduced the progress checker failure observed during the manual outbound journey.

The return journey was not included in the controlled outbound repeatability dataset because the route direction and starting position differed.

#### Controlled Navigation Experimental Configuration

Three independent outbound navigation trials were performed using the same recorded software configuration.

| Parameter | Value |
|---|---|
| Robot count | 1 |
| Robot namespace | `j100_0000` |
| World | Warehouse |
| Gazebo GUI | Enabled |
| RViz2 | Disabled |
| Open-RMF | Not running |
| Initial physical X | 0.0 m |
| Initial physical Y | 0.0 m |
| Initial physical Z | 0.0635 m |
| Initial physical yaw | 0.0 rad |
| Goal X | -1.2874 m |
| Goal Y | -3.1625 m |
| Goal yaw | 0.0 rad |
| Goal frame | `map` |
| Global planner | NavFn |
| Local controller | MPPI |
| Navigation timeout | 120 s |
| Telemetry target frequency | 10 Hz |
| Resource sampling interval | 2 s |
| Settling observation period | 2 s |
| Independent repetitions | 3 |

Simulation, localization, and navigation were restarted before each trial.

The physical starting pose was verified independently using Gazebo.

AMCL was initialized at the original spawn position before starting autonomous navigation.

Configuration verification confirmed:

- Identical hashes for all eight monitored configuration and source-code files.
- Identical recorded Docker image identifiers.
- Identical recorded experimental configuration values.
- Distinct container startup timestamps for simulation, localization, and navigation.
- No reported experiment runner errors.
- Identical measured physical starting positions and orientations.
- Initial estimated position variation below 1 mm.

The files included in configuration hash verification were:

```text
compose.yaml
compose.visualization.yaml
compose.navigation.yaml
config/jackal/robot.yaml
config/navigation/j100/nav2.yaml
docker/nav2/Dockerfile
scripts/navigation_probe.py
scripts/run_navigation_trial.py
```

Simulation warm-up duration was not standardized. Trial 1 began substantially later in simulation time than trials 2 and 3.

#### Individual Navigation Results

| Metric | Trial 1 | Trial 2 | Trial 3 |
|---|---:|---:|---:|
| Planning result | SUCCEEDED | SUCCEEDED | SUCCEEDED |
| Navigation result | SUCCEEDED | SUCCEEDED | SUCCEEDED |
| Navigation duration (s) | 64.301 | 65.171 | 65.029 |
| Navigation simulation duration (s) | 64.260 | 65.178 | 64.953 |
| Nav2 planning time (ms) | 3 | 6 | 3 |
| Client planning duration (ms) | 9.104 | 10.642 | 9.149 |
| Planned distance (m) | 6.0615 | 6.0615 | 6.0615 |
| Odometry trajectory length (m) | 6.5479 | 6.6535 | 6.6363 |
| Localized trajectory length (m) | 6.9155 | 7.1004 | 7.0441 |
| Estimated position error (m) | 0.2440 | 0.2433 | 0.2605 |
| Ground-truth position error (m) | 0.2994 | 0.3163 | 0.4071 |
| Recovery count | 1 | 1 | 1 |
| Final stationary state | Yes | Yes | Yes |

All three trials generated the same initial global path length at full recorded precision.

#### Aggregate Navigation Performance

The following statistics use the three successful controlled navigation trials.

| Metric | Mean | Sample standard deviation |
|---|---:|---:|
| Server planning time (ms) | 4.000 | 1.732 |
| Client planning duration (ms) | 9.632 | 0.875 |
| Planning acceptance latency (ms) | 0.757 | 0.053 |
| Navigation acceptance latency (ms) | 0.443 | 0.054 |
| Navigation wall duration (s) | 64.834 | 0.467 |
| Navigation simulation duration (s) | 64.797 | 0.478 |
| Effective navigation RTF | 0.999434 | 0.000643 |
| Planned path length (m) | 6.061451 | 0.000000 |
| Planned-to-direct distance ratio | 1.799589 | 0.000000 |
| Odometry trajectory length (m) | 6.612580 | 0.056662 |
| Sampled localized trajectory length (m) | 7.019984 | 0.094759 |
| Odometry-to-planned distance ratio | 1.090924 | 0.009348 |
| Average odometry speed including settling (m/s) | 0.098995 | 0.000154 |
| Mean absolute linear velocity (m/s) | 0.101850 | 0.000111 |
| Action feedback frequency (Hz) | 99.302 | 1.179 |
| Telemetry sampling frequency (Hz) | 9.849 | 0.011 |
| Per-trial telemetry interval SD (ms) | 1.306 | 0.888 |

The mean executed odometry trajectory was approximately 9.09% longer than the initial global path.

The trajectory difference may include replanning, recovery behavior, and deviations during local control.

The effective navigation RTF is calculated using the action client's measured simulation duration divided by its measured wall duration.

This measurement is distinct from the Gazebo statistics-based RTF distribution.

#### Positioning Accuracy

Navigation completion and physical positioning accuracy are evaluated independently.

| Metric | Mean | Sample standard deviation |
|---|---:|---:|
| Estimated position error (m) | 0.249294 | 0.009697 |
| Estimated yaw error (rad) | 0.019492 | 0.008487 |
| Ground-truth position error (m) | 0.340943 | 0.057940 |
| Ground-truth yaw error (rad) | 0.065151 | 0.014788 |
| Localization position discrepancy (m) | 0.137086 | 0.034670 |
| Localization yaw discrepancy (rad) | 0.045659 | 0.017677 |

Ground-truth position and orientation were obtained directly from Gazebo before and after navigation.

Direct comparison between estimated map-frame poses and Gazebo world-frame poses assumes that the coordinate systems are aligned.

This assumption is explicitly recorded in the analysis results and has not been independently calibrated.

The final position tolerance is 0.25 m, and the final yaw tolerance is 0.25 rad.

| Success Criterion | Result |
|---|---:|
| Planning action success | 3/3 (100%) |
| Navigation action success | 3/3 (100%) |
| Stationary after completion | 3/3 (100%) |
| Final estimated pose tolerance compliance | 2/3 (66.67%) |
| Final ground-truth pose tolerance compliance | 0/3 (0%) |

All three trials reached successful navigation action completion.

However, none of the final physical positions satisfied the configured position tolerance under the map/world alignment assumption.

Trial 3 also exceeded the estimated position tolerance after the settling observation period.

Its final estimated position error was approximately 0.2605 m.

Because Nav2 evaluates successful completion before the probe's final settling measurement, subsequent residual motion or localization corrections may contribute to the discrepancy. The exact contribution of each mechanism has not been isolated.

These findings demonstrate the importance of distinguishing action-level success from final positioning accuracy.

#### Recurring Progress Checker Failure

A progress checker failure occurred during all five obstacle-navigation executions, including the manual outbound test, the automated return test, and the three controlled outbound trials.

All executions subsequently completed successfully.

The recurring sequence was:

```text
Progress checker failure
        |
        v
FollowPath action abort
        |
        v
Local costmap clearing
        |
        v
Navigation resumes
        |
        v
Goal succeeds
```

The controlled trials exhibited the following behavior:

| Metric | Trial 1 | Trial 2 | Trial 3 |
|---|---:|---:|---:|
| Failure time after navigation start (s) | 21.571 | 22.471 | 21.749 |
| Configured movement radius (m) | 0.500 | 0.500 | 0.500 |
| Approximate diagnostic window (s) | 10 | 10 | 10 |
| Net odometry displacement (m) | 0.1263 | 0.0812 | 0.1332 |
| Sampled odometry distance (m) | 0.1788 | 0.2206 | 0.2385 |
| Mean absolute linear velocity (m/s) | 0.0188 | 0.0227 | 0.0237 |
| Mean absolute angular velocity (rad/s) | 0.1077 | 0.1133 | 0.1086 |

The diagnostic windows show that the robot continued moving slowly and rotating while achieving limited translational displacement.

This behavior is consistent with the configured progress checker failure conditions.

However, the diagnostic windows are approximately aligned using navigation telemetry timestamps and server log events. They do not reproduce the internal progress checker's exact reference pose or reset timing.

The underlying cause has not been isolated.

The failure occurred both with and without RViz2 enabled, and it also occurred during a trial with no recorded controller frequency warnings.

The recurring behavior is therefore retained as a single-robot baseline finding rather than attributed to multi-robot scalability or a specific computational bottleneck.

No navigation parameters were changed to suppress the recovery behavior before completing the controlled experiments.

#### Navigation Reliability

| Reliability Metric | Total Across Three Trials |
|---|---:|
| Navigation action failures | 0 |
| Progress checker failures | 3 |
| Recorded recoveries | 3 |
| Local costmap clearing events | 3 |
| Global costmap clearing events | 0 |
| Controller frequency warnings | 3 |
| Local costmap TF rejection messages | 2 |
| Global costmap TF rejection messages | 2 |
| Missing navigation localization samples | 0 |
| Missing navigation odometry samples | 0 |
| Manual interventions required | 0 |

The controller frequency warnings included reported instantaneous rates of approximately 13.3333 Hz, 333.3333 Hz, and 5.4645 Hz.

The anomalously high frequency value is retained as a timing observation and is not interpreted as sustained controller execution throughput.

The collected log counts describe recorded messages rather than independently verified totals of all underlying events.

Startup warnings occurring before the experiment log collection window are excluded from these counts.

#### Computational Resource Benchmark

A host-side navigation trial runner was developed to coordinate:

- Initial Gazebo ground-truth collection.
- Navigation probe execution.
- Continuous Gazebo RTF collection.
- Independent Docker resource measurements.
- System-wide NVIDIA GPU measurements.
- Final Gazebo ground-truth collection.
- Navigation logs.
- Configuration hashes and image identifiers.
- Experimental metadata.

Resource measurements are filtered to the approximate navigation execution window.

This excludes most instrumentation warm-up and settling activity from the resource statistics.

The three controlled trials produced 94 resource samples and 1,904 RTF samples within their navigation measurement windows.

The aggregate resource statistics assign equal weight to each independent trial rather than pooling every sample into a single distribution.

##### CPU Consumption

| Component | Mean CPU (%) | Sample standard deviation (%) | Maximum observed sample (%) |
|---|---:|---:|---:|
| Simulation | 169.01 | 4.71 | 231.79 |
| Navigation | 89.68 | 3.50 | 132.41 |
| Localization | 11.83 | 0.58 | 19.48 |
| Combined | 270.52 | 8.75 | Not calculated |

Docker CPU percentages are expressed relative to the capacity of one logical CPU.

The combined mean CPU consumption corresponds to approximately 2.705 logical CPU cores' worth of processing capacity.

On the current 12-logical-CPU host, this corresponds to approximately 22.54% of total logical CPU capacity.

This calculation excludes other host processes and does not imply that every CPU core is equally utilized.

The relative shares of measured container CPU consumption were approximately:

| Component | Share |
|---|---:|
| Simulation | 62.48% |
| Navigation | 33.15% |
| Localization | 4.37% |

Navigation container measurements include the navigation instrumentation probe.

##### Memory Consumption

| Component | Mean RAM (MiB) | Sample standard deviation (MiB) | Maximum observed sample (MiB) |
|---|---:|---:|---:|
| Simulation | 963.17 | 4.96 | 1028.10 |
| Navigation | 379.96 | 1.53 | 384.00 |
| Localization | 104.74 | 0.32 | 106.00 |
| Combined | 1447.86 | 3.69 | Not calculated |

The combined mean container memory measurement was approximately 1.414 GiB.

Combined CPU and RAM statistics are calculated from each trial's summed component means. Component maxima are not added to estimate a combined peak because individual maxima may occur at different times.

##### GPU Consumption

| Metric | Mean | Sample standard deviation | Maximum observed sample |
|---|---:|---:|---:|
| GPU utilization (%) | 33.62 | 4.91 | 39.00 |
| GPU memory utilization (%) | 22.02 | 3.68 | 25.00 |
| VRAM usage (MiB) | 1524.75 | 37.00 | 1580.00 |
| GPU temperature (degrees C) | 49.08 | 5.53 | 55.00 |
| GPU power consumption (W) | 23.67 | 0.22 | 32.95 |

GPU measurements are system-wide and cannot be attributed exclusively to Gazebo.

GPU memory utilization describes memory activity, not the percentage of VRAM capacity allocated.

GPU utilization varied between trials despite relatively consistent CPU and RAM consumption. Rendering conditions and other desktop activity were not independently controlled or measured.

##### Gazebo Real-Time Factor

| Metric | Result |
|---|---:|
| Independent trials | 3 |
| Navigation-window RTF samples | 1904 |
| Mean of trial mean RTF values | 0.999655 |
| Between-trial sample standard deviation | 0.000732 |
| Minimum observed RTF sample | 0.570732 |
| Maximum observed RTF sample | 1.204473 |

Gazebo maintained approximately real-time execution across all three navigation experiments.

Temporary slowdowns and variations in reported RTF were observed despite an overall mean close to 1.0.

The minimum RTF sample does not establish the duration of the corresponding slowdown.

#### Startup and Infrastructure Observations

The navigation integration identified several infrastructure issues that remain relevant for multi-robot development.

**1. Initial localization**

AMCL requires an initial pose estimate corresponding to the robot's starting position.

Explicit initial-pose publication was used for all controlled trials.

Future multi-robot startup procedures must initialize each robot independently.

**2. Delayed TF discovery**

Navigation startup occasionally encountered unavailable `odom` or `map` transformations.

Some warnings resolved during initialization, while an earlier startup attempt required restarting the navigation service after localization was established.

Initial AMCL pose availability and `odom → base_link` transform availability are distinct startup dependencies.

Lifecycle activation should be verified before navigation experiments begin.

**3. Intermittent sensor transform rejection**

Some LiDAR messages were rejected by costmap TF message filters because their timestamps preceded the available transform cache.

These events occurred intermittently during both initialization and subsequent operation.

Their underlying cause has not been isolated.

**4. BehaviorTree.CPP compatibility**

The navigation image requires a compatible set of Nav2 and BehaviorTree.CPP packages.

The custom Dockerfile explicitly upgrades the relevant packages, but exact package-version pinning and publication of the validated image remain future reproducibility improvements.

**5. Container shutdown**

Simulation and visualization have repeatedly required forced termination after the requested Docker shutdown timeout.

Exit code `137` was observed following these stop operations.

Signal propagation and orderly container shutdown should be investigated separately from navigation execution reliability.

**6. Planning simulation-time anomaly**

During controlled trial 1, the planning action's client-observed simulation duration was 267 ms, while the measured wall duration was approximately 9.104 ms and the server-reported planning time was 3 ms.

Delayed simulation clock delivery or callback scheduling may contribute to this discrepancy.

The raw measurement is retained. Client wall duration and server-reported planning time are used for the primary planning latency comparisons.

#### Experimental Limitations

- Only one robot and one controlled outbound navigation route were evaluated.
- Three repetitions provide preliminary repeatability measurements rather than statistically established reliability limits.
- All controlled navigation trials used the Gazebo graphical interface.
- GPU measurements are system-wide and include unrelated GPU activity.
- Navigation CPU measurements include the experimental instrumentation probe.
- Docker resource measurements are sampled rather than continuous.
- Resource sampling windows are approximately aligned using navigation telemetry timestamps.
- Resource window alignment assumes compatible host and container monotonic clock bases.
- The trial runner does not independently verify every navigation lifecycle state before execution; lifecycle readiness is also checked during environment preparation.
- Simulation warm-up duration varied between trials.
- Configuration verification covers recorded files, image identifiers, and metadata rather than every possible runtime condition.
- Gazebo ground truth was collected only at the beginning and end of each navigation experiment.
- Continuous physical trajectory length was not measured independently.
- Raw odometry trajectory length may contain odometry estimation errors.
- Map-frame trajectory length may include localization corrections.
- Direct map/world pose comparisons assume coordinate alignment.
- Progress checker diagnostic windows approximate the relevant failure intervals.
- The internal progress checker reference positions were not recorded.
- Action feedback reception rates include the effects of callback scheduling and instrumentation.
- Final stationary state is evaluated using odometry, with additional settling samples providing supporting evidence.
- Independent warehouse physics collision verification remains outstanding.
- The causes of recurring progress failures, localization discrepancies, and intermittent timing warnings have not been isolated.
- No conclusions about multi-robot scalability can yet be drawn.

#### Scripts and Datasets

Relevant navigation scripts:

| Script | Purpose |
|---|---|
| `scripts/navigation_probe.py` | Planning, navigation action execution, and telemetry collection |
| `scripts/analyze_navigation_probe.py` | Individual navigation experiment analysis |
| `scripts/analyze_navigation_repeat.py` | Navigation repeatability and reliability aggregation |
| `scripts/run_navigation_trial.py` | Coordinated navigation and resource benchmark execution |
| `scripts/analyze_navigation_resources.py` | Resource aggregation and recorded configuration verification |

Supporting collection scripts:

- `scripts/collect_resources.py`
- `scripts/run_baseline.py`

The navigation runner reuses existing collection utilities without modifying the previous idle simulation benchmark.

Manual navigation validation:

```text
results/raw/obstacle_planning_smoke.txt
results/raw/obstacle_navigation_smoke.txt
```

Automated smoke tests:

```text
results/raw/navigation_probe_planning_smoke_1robot_20261120_125102/
results/raw/navigation_probe_execution_smoke_1robot_20261120_125340/
```

Controlled navigation repeatability dataset:

```text
results/raw/navigation_repeat_1robot_20261120_132608/
```

Dataset organization:

```text
navigation_repeat_1robot_20261120_132608/
├── aggregate.json
├── resource_aggregate.json
├── trial_01/
├── trial_02/
└── trial_03/
```

Each controlled trial contains:

| File | Contents |
|---|---|
| `summary.json` | Raw navigation probe summary |
| `planned_path.csv` | Initial global path geometry |
| `samples.csv` | Timestamped navigation telemetry |
| `analysis.json` | Individual experiment analysis |
| `ground_truth_initial.txt` | Initial Gazebo model pose |
| `ground_truth_final.txt` | Final Gazebo model pose |
| `navigation.log` | Navigation server logs |
| `probe.log` | Navigation probe output |
| `rtf.csv` | Gazebo statistics measurements |
| `resources.csv` | Container and GPU resource measurements |
| `resource_summary.json` | Navigation-window resource statistics |
| `metadata.json` | Experiment configuration and execution metadata |
| `gazebo_stderr.log` | Gazebo statistics collector standard error |

The aggregate files contain navigation performance, reliability statistics, resource measurements, and experimental configuration verification results.

All three successful trials are retained, including trials with recovery events, positioning discrepancies, and timing anomalies.

#### Conclusion

The automated single-robot baseline establishes reference measurements for simulation performance, computational resource consumption, sensor timing, DDS communication, motion-control responsiveness, localization, and autonomous navigation.

The single Jackal successfully completed all three controlled navigation trials while maintaining approximately real-time simulation performance.

However, the experiments identified recurring recovery behavior and differences between navigation action success and final positioning accuracy.

These findings are part of the baseline and will be considered when interpreting subsequent multi-robot experiments.

The next development stage introduces a second independently controlled and localized Jackal into the same simulation environment.

Open-RMF integration will follow independent multi-robot functional validation.

## 6. Current Project Status

| Milestone | Status |
|---|---|
| 0 — Repository Initialization | Completed |
| 1 — Container Environment Configuration | Completed |
| 2 — Single-Robot Functional Validation | Completed |
| 3A — RTF and Resource Monitoring | Completed |
| 3B — Automated Sensor Monitoring | Completed |
| 3C — ROS 2 Graph Validation | Completed |
| 3D — DDS Communication Benchmarking | Completed |
| 3E — Automated Motion-Control Benchmarking | Completed |
| 3F — Independent RViz2 Visualization | Completed |
| 3G — Nav2 Autonomous Navigation and Benchmarking | Completed |
| 4A — Two-Robot Simulation | Next |
| 4B — Open-RMF Fleet Coordination | Planned |
| 4C — Incremental Fleet Scalability | Planned |
| 5 — Bottleneck Investigation | Planned |
| 6 — Analysis and Research Report | Planned |

**Next objective:** Introduce a second Jackal into the existing warehouse simulation and validate independent robot identities, namespaces, sensor streams, control interfaces, localization, and autonomous navigation.

The two-robot development stage will also establish reusable initialization procedures for robots with different physical starting positions.

Independent two-robot operation will be validated before introducing Open-RMF fleet coordination.

## 7. Reproducibility and Future Deliverables

The project will progressively include:

- Version-controlled robot configurations.
- Docker Compose configurations.
- Custom Dockerfiles when additional software is required.
- Versioned Docker images published to a container registry.
- Automated experiment scripts.
- Raw measurement datasets.
- Processed experimental results.
- Performance graphs and visualizations.
- Documented experimental limitations.
- Reusable multi-robot initialization and lifecycle management.
- Standardized experiment warm-up and reset procedures.
- Continuous independent ground-truth monitoring.
- Fleet-level navigation and coordination benchmarks.

Results will be added as experiments are completed and validated.

The current single-robot datasets will be preserved as reference measurements for examining changes in simulation performance, communication latency, robot responsiveness, navigation reliability, and computational resource requirements as the simulated fleet grows.

## 8. Upstream Projects

This research builds on the official Clearpath Robotics software ecosystem and the ROS 2 navigation ecosystem.

**Clearpath Simulator:**
https://github.com/clearpathrobotics/clearpath_simulator

**Clearpath Docker:**
https://github.com/clearpathrobotics/clearpath_docker

**Clearpath Configuration:**
https://github.com/clearpathrobotics/clearpath_config

**Clearpath Nav2 Demos:**
https://github.com/clearpathrobotics/clearpath_nav2_demos

**Navigation2:**
https://github.com/ros-navigation/navigation2

**Gazebo:**
https://gazebosim.org/

**Open-RMF:**
https://github.com/open-rmf

The investigation is independent and does not represent an official Clearpath Robotics, Nav2, Gazebo, or Open-RMF benchmark.
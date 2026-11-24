# Clearpath Scalability

**ROS 2 Jazzy | Gazebo Harmonic | Clearpath Jackal J100 | Nav2 | Open-RMF | Docker**

An experimental robotics engineering project investigating multi-robot simulation, autonomous navigation, fleet coordination, and performance measurement.

The project uses Clearpath Jackal J100 robots simulated in Gazebo, with independent ROS 2 navigation stacks and Open-RMF fleet coordination.

The long-term objective is to investigate how simulation performance and robot reliability change as the number of robots increases, identify bottlenecks, and evaluate potential engineering improvements.

## Project Status

**Development paused — November 2026**

The initial development phase established functional single-robot and two-robot baselines.

Open-RMF integration was implemented and tested successfully for task dispatch, traffic coordination, and RMF-directed navigation.

Preliminary synchronized benchmarking infrastructure was also developed.

However, a complete, repeatable Open-RMF task-performance benchmark has not yet been established.

Development will resume in the future.

The existing validated configurations, Git tags, experimental scripts, and locally retained measurements will serve as the starting point for further investigation.

## Validated Achievements

### 1. Single-Robot Simulation and Navigation

Implemented and validated:

- Clearpath Jackal J100 simulation in Gazebo Harmonic.
- ROS 2 Jazzy communication and robot control.
- LiDAR, odometry, and simulated inertial measurements.
- AMCL localization.
- Nav2 autonomous navigation.
- Automated performance measurements.
- Independent RViz2 visualization.

Three controlled navigation trials completed successfully.

The experiments also revealed positioning discrepancies and recurring navigation recovery behavior that require further investigation.

### 2. Two-Robot Simulation

Successfully extended the environment to two Jackal J100 robots.

Validated:

- Independent robot namespaces.
- Independent localization and navigation stacks.
- Simultaneous robot operation.
- Navigation goal execution.
- Robot-state monitoring.
- Reproducible simulation initialization.

The validated configuration is preserved in the Git tag:

`baseline-nav2-2robots`

### 3. Open-RMF Integration

Integrated Open-RMF with the existing Clearpath simulation and Nav2 architecture.

Implemented and validated:

- Open-RMF traffic scheduling.
- Two-robot fleet registration.
- Warehouse navigation graph.
- Python fleet adapter.
- RMF task bidding and dispatch.
- Navigation through the existing Nav2 interfaces.
- RMF traffic coordination.
- Controlled navigation goal replacement.

Two patrol requests were accepted and dispatched successfully during subsequent functional testing.

The assigned robot executed navigation commands, and Nav2 reported successful navigation results.

The validated two-robot RMF configuration is preserved in:

`baseline-rmf-2robots`

**Limitation:** An authoritative Open-RMF terminal task-completion event was not captured during these tests. Successful navigation and task release should not be interpreted as independent proof of RMF task completion.

### 4. Performance Monitoring

Developed experimental tools for:

- Docker container CPU and memory measurements.
- Linux cgroup resource sampling.
- Host CPU and memory measurements.
- GPU utilization monitoring.
- Gazebo real-time factor measurements.
- Open-RMF event collection.
- Synchronized measurement windows.
- Experimental metadata and integrity checks.

A synchronized two-robot idle-monitoring experiment successfully validated the unified collection infrastructure.

Active fleet-workload benchmarking remains unfinished.

## Technical Findings

### Navigation Reliability

Single-robot experiments identified recurring progress-checker failures followed by successful recovery.

Navigation action success did not always correspond to the configured final positioning-accuracy requirement.

These observations demonstrate the importance of measuring navigation success, positioning accuracy, and recovery behavior separately.

### ROS 2 and Fleet Integration

Independent robot namespaces, localization, Nav2 interfaces, and fleet registration were successfully implemented.

Traffic coordination and navigation goal replacement were validated in the two-robot configuration.

These results establish functional operation, but do not establish scalability beyond two robots.

### Performance Measurement

Synchronized measurements of simulator performance, container resources, and RMF events were demonstrated.

The monitoring infrastructure provides a foundation for future controlled scalability experiments.

### Open-RMF Task-State Monitoring

The installed Open-RMF 2.7.2 environment did not expose the expected ROS 2 task-state update topics.

An experimental Python task-update listener was registered successfully but did not receive callbacks during the controlled patrol tests.

A potential solution using Open-RMF's WebSocket monitoring interface was investigated but not implemented or validated.

This remains an outstanding requirement before official task-throughput and task-completion benchmarks can be conducted.

## Current Limitations

The following work remains incomplete:

- Authoritative Open-RMF task-completion monitoring.
- Repeated, standardized RMF workload experiments.
- Comprehensive two-robot RMF performance benchmarking.
- Scaling to three or more Clearpath robots.
- Isolation of simulator, navigation, DDS, and RMF performance bottlenecks.
- Investigation of outstanding positioning and navigation recovery anomalies.

The existing results should not be interpreted as evidence of performance at larger fleet sizes.

## Future Development

When development resumes, the next priorities will be to establish reliable Open-RMF task-outcome monitoring, implement standardized fleet workloads, and validate repeatable performance measurements.

Subsequent experiments may investigate configurations with 3, 5, 10, and additional simulated robots, subject to available hardware and measured stability.

Potential bottlenecks will be analyzed individually rather than assuming that all performance degradation originates in Gazebo or Open-RMF.

## Repository Structure

| Directory | Purpose |
|---|---|
| `config/` | Robot, navigation, simulation, and RMF configurations |
| `docker/` | Custom Docker images and environment configuration |
| `scripts/` | Simulation, navigation, validation, and benchmark utilities |
| `docs/` | Technical documentation and experimental findings |
| `results/` | Experimental outputs and processed measurements |
| `compose*.yaml` | Docker Compose service definitions |

Raw experimental data may be retained locally rather than included in the public repository.

## Historical Documentation

The original, detailed project README has been preserved as:

[Historical development documentation](docs/historical-readme-2026-11.md)

It includes the earlier setup procedures, navigation experiments, detailed measurements, and development methodology.

Some historical milestone descriptions and commands reflect the configuration at the time they were written and may not represent the project's latest state.

## Validated Git Baselines

**Nav2 two-robot baseline**

`baseline-nav2-2robots`

**Open-RMF two-robot baseline**

`baseline-rmf-2robots`

Both tags preserve previously validated development checkpoints.

## Research Scope

This is an independent experimental project using publicly available robotics software.

It is not an official benchmark published by Clearpath Robotics, Open-RMF, Gazebo, or the ROS 2 development teams.

The project is preserved for future research, development, and experimentation.
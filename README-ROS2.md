# PyContract ROS2 Package

## ROS2 Build

Make sure ROS2 Jazzy or later is installed.

```bash
# The project repository can be used as a ROS workspace
source /opt/ros/<ros2-distro>/setup.bash or setup.zsh

# The list of packages:
colcon graph
# Will produce:
# pycontract           +**
# examples_ros2         + 
# pycontract_examples    +

# Build all the packages in this repository
colcon build

# Source the installed packages
source install/setup.bash
```

## ROS2 Usage

### ROS2 Examples

This repository provides example ROS2 nodes for testing and monitoring contract-based systems using pycontract.

#### run_monitor (ROS2 equivalent of [examples/run_monitor.py](./examples/run_monitor.py))

```shell
ros2 launch examples_ros2 test_monitor.launch.py
```

The above will result in the following interactions:

```mermaid
sequenceDiagram
    autonumber
    participant ScenarioPublisher
    participant ROS2_RunMonitor
    participant ROS2_Middleware as ROS 2 Topics

    %% step 0
    ScenarioPublisher->>ROS2_Middleware: publish A(x=42) on /event_a
    ROS2_Middleware->>ROS2_RunMonitor: A(x=42)
    ROS2_RunMonitor-->>ROS2_RunMonitor: process A

    %% step 1
    ScenarioPublisher->>ROS2_Middleware: publish B(x=42) on /event_b
    ROS2_Middleware->>ROS2_RunMonitor: B(x=42)
    ROS2_RunMonitor-->>ROS2_RunMonitor: process B

    %% step 2
    ScenarioPublisher->>ROS2_Middleware: publish C(x=42) on /event_c
    ROS2_Middleware->>ROS2_RunMonitor: C(x=42)
    ROS2_RunMonitor-->>ROS2_RunMonitor: process C

    %% step 3 – terminate
    ScenarioPublisher->>ROS2_Middleware: publish End() on /end
    ROS2_Middleware->>ROS2_RunMonitor: End()
    ROS2_RunMonitor-->>ROS2_RunMonitor: monitor.end(); rclpy.shutdown()
    ROS2_RunMonitor-->>ScenarioPublisher: node exits
    ScenarioPublisher-->>ScenarioPublisher: finished cleanly
`
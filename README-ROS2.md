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

#### 1. Start the ROS 2 Core

In a new terminal:
```bash
ros2 daemon start
```

#### 2. Run the Example Nodes

Open a new terminal for each node (after sourcing your workspace):

- **Command Publisher**
  
  Publishes commands to the system:
  ```bash
  ros2 run examples_ros2 command_publisher.py
  ```

- **Status Publisher**
  
  Publishes status updates:
  ```bash
  ros2 run examples_ros2 status_publisher.py
  ```

- **ROS2 Monitor**
  
  Monitors contract properties and system status:
  ```bash
  ros2 run examples_ros2 ros2_monitor.py
  ```

#### 3. Visualize or Inspect Topics

You can inspect published topics using:
```bash
ros2 topic list
ros2 topic echo /<topic_name>
```

Make sure to open a new terminal and source your workspace (`source install/setup.bash`) before running each node.

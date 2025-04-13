# Puzzlebot-Challenge
ROS2 Package for controlling a differential drive robot.

## Packages

### Controllers
The `controllers` package contains two essential nodes responsible for controlling and monitoring the robot’s movement and position. These nodes are:

1. **controller_node**: Implements a PID Controller for managing the robot’s position.
2. **odometry_node**: Computes the robot's odometry based on wheel velocity.

### Nodes in the Controllers Package

#### 1. `controller_node`
- **Node Name**: `controller_node`
- **ROS2 Command to Run the Node**:
  ```bash
  ros2 run controllers controller_node

#### 2. 'odometry_node'
- **Node Name**: `odometry_node`
- **ROS2 Command to Run the Node**:
  ```bash
  ros2 run controllers odometry_node

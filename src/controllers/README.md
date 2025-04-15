# Controllers Package
This ROS package includes multiple nodes that deal with different control methods for the Puzzlebot.

## Launch files
- **Dead Reckoning:** `Dead Reckoning`
- - **Info:** Initializes `controller_node` and `odometry_node` to track robot location
- - **Usage:** Interacts with the `/setpoint` to read next desired setpoint and head to that location
   
## Node Information

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

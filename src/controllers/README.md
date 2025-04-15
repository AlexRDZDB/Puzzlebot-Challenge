# Controllers Package
This ROS package includes multiple nodes that deal with different control methods for the Puzzlebot.

## Launch files
- **Dead Reckoning:** `Dead Reckoning`
   - **Info:** Initializes `controller_node` and `odometry_node` to track robot location
   - **Usage:** Interacts with the `/setpoint` to read next desired setpoint and head to that location
   
## Node Information

#### 1. `controller_node`
- **Node Name**: `controller_node`
- **Node Description:** PID Controller for a differential drive robot. Subscribes to `/setpoint` topic to obtain desired pose and the `/odom` topic to obtain current pose
- **ROS2 Command to Run the Node**:
  ```bash
  ros2 run controllers controller_node

#### 2. `odometry_node`
- **Node Name**: `odometry_node`
- **Node Description:** Subscribes to encoder angular velocity values to estimate robot position by using Dead Reckoning.
- **ROS2 Command to Run the Node**:
  ```bash
  ros2 run controllers odometry_node

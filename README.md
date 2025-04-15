# Puzzlebot-Challenge
ROS2 Package for controlling a differential drive robot.

## Packages

### Controllers
The `controllers` package contains two essential nodes responsible for controlling and monitoring the robot’s movement and position. These nodes are:

1. **controller_node**: Implements a PID Controller for managing the robot’s position.
2. **odometry_node**: Computes the robot's odometry based on wheel velocity.

# Robotcraft
A repository of the tasks that we work on during RobotCraft 2021.

## ROS Simulation
We are using the wall following algorithm proposed on this website: https://www.theconstructsim.com/wall-follower-algorithm/ to maneuver the robot out of the maze.

To improve this algorithm, we additionally use the information from the odom topic, namely the robot's rotation (the angular z). Before we go straight, we make sure that the robot has been rotated exactly 90° in order to prevent it from colliding with the wall.

Even though it currently moves a bit wobbly (could be improved by playing around with the parameters?), the algorithm seems to work quite well.

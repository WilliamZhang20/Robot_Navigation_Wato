# Robot Car Navigation

Part of the WATOnomous Autonomous Software Division Admissions Assignment.

The goal is to allow a robot car in a room full of blocks to navigate from A to B while avoiding obstacles.

Implemented using nodes in ROS2. Each node is a data endpoint, collecting data from a sensor/another node, and sending that information to another node for processing, or to the robot for control actuation.

There are four components, each is a ROS2 Node:
1. Local Costmap Generation from LiDAR signals.
2. Global Costmap / Occupancy Grid assembly by stitching local costmaps together.
3. Global Path Planning from source to destination points using the A* algorithm.
4. Local Planning to take the A* algorithm's path and actuate the robot's velocity controls to follow it. Original implementation is Pure Pursuit, adjusting to Timed-Elastic-Band (TEB) based Model-Predictive Control
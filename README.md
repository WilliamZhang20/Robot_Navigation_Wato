# Robot Car Navigation

Part of the Watonomous Autonomous Software Division Admissions Assignment.

The goal is to allow a robot car in a room full of blocks to navigate from A to B while avoiding obstacles.

Implemented using nodes in ROS2. Each node is a data endpoint, collecting data from a sensor/another node, and sending that information to another node for processing, or to the robot for control actuation.

There are four components, each is a ROS2 Node:
1. Local Costmap Generation from LiDAR signals.
2. Global Costmap / Occupancy Grid assembly by stitching local costmaps together.
3. Global Path Planning from source to destination points using the A* algorithm.
4. Local Planning to take the A* algorithm's path and actuate the robot's velocity controls to follow it. Original implementation is Pure Pursuit, adjusting to Timed-Elastic-Band (TEB) based Model-Predictive Control

## Getting Started

To run, it takes only two very simple steps from the home directory of the repository:
1. Run `./watod build` and then `./watod up`
2. Open a Foxglove server to the port indicated in the terminal.

### Notes on Timed Elastic Band (TEB) Algorithm for Velocity Control

Additional extension work was done while a research assistant at [CL2](https://github.com/CL2-UWaterloo).

Earliest concept is from Quinlan & Khatib at Stanford in 1994, with Sean Quinlan's PhD dissertation [here](http://i.stanford.edu/pub/cstr/reports/cs/tr/95/1537/CS-TR-95-1537.pdf). 

Original paper is [here](https://ieeexplore.ieee.org/document/6309484) from 2012, by researchers at TU Dortmund, principally Christoph Rosmann.

Those researchers have put their code implementation into a ROS Package [here](https://github.com/rst-tu-dortmund/teb_local_planner), from which I heavily drew inspiration.

An extension in which multiple trajectories are planned (a variation called Homotopy class planning) is [here](https://rst.etit.tu-dortmund.de/storages/rst-etit/r/Global/Paper/Roesmann/2015_Roesmann_ECMR.PDF).

A further idea combining TEB with MPC is [here](https://rst.etit.tu-dortmund.de/storages/rst-etit/r/Global/Paper/Roesmann/2015_Roesmann_ECC.PDF).

An additional mini-extension for band flexibility and stability is from 2017 found [here](https://www.mi.fu-berlin.de/inf/groups/ag-ki/publications/Elastic-Bands/03212.pdf). 

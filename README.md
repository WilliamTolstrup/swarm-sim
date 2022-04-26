# Simulation of **swarm robotics** with multiple robots navigation a map with the global planner separated

The goal is to compute *optimal* * global paths on a remote computer and send the paths to the individual robots wirelessly through *5G networking* *

*Not the goal of this project

## Packages

**nav2_gazebo_spawner** creates the robots in gazebo with user arguments consisting of:
- Robot name
- Robot namespace
- Positional arguments (x,y,z)
- Timeout (default 10 seconds)

Custom arguments can be made to this file if it is required.

**pc** Launches Rviz and displays each robot. It also executes the global planner node required for navigation.

**remote** Launches the robots in gazebo, calling everything required from the Navigation2 package 


## Requirements

Before launching these packages, *Navigation2*, *Nav2-bringup*, and, for the time being, *turtlebot3* must be installed as Debian packages:
    `sudo apt install ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-turtlebot3*`

To correct world models are included in the folder *gazebo_models*. These needs to be in the hidden .gazebo/models folder. Copy/paste the following:

    `sudo cp -r swarm-sim/gazebo_models/workcell/ /home/$USER/.gazebo/models`

    `sudo cp -r swarm-sim/gazebo_models/aws_robomaker_warehouse_Bucket_01/ /home/$USER/.gazebo/models`

    `sudo cp -r swarm-sim/gazebo_models/aws_robomaker_warehouse_PalletJackB_01/ /home/$USER/.gazebo/models`
    
    `sudo cp -r swarm-sim/gazebo_models/aws_robomaker_warehouse_ShelfE_01/ /home/$USER/.gazebo/models`
    
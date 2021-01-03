# turtlebot\_control

This repository contains the ROS-powered gazebo simulation for a Burger Turtlebot. Its extension (Android app) for a remote control of the Turtlebot's simulated movements can be found [here](https://github.com/emilia-szymanska/android_UDP_control).

## Installation

The repository was built for Ubuntu 20.04 with ROS Noetic and Python3.8.

Clone this repo by running:
```
git clone https://github.com/emilia-szymanska/turtlebot_control.git
```

After installing ROS, in each new terminal, run
```
source /opt/ros/<distro>/setup.bash
```
where you replace `<distro>` with the installed distribution, e.g. noetic.

To source the workspace, run (from the root of this repository): 
```
source devel/setup.bash
```

## Running the pipeline

To run the whole project, put the following command in a terminal:
```
roslaunch turtlebot3_control turtlebot3_app_control.launch
```

## Contents    

The package `turtlebot3_control` contains:

- `UDP_server` node (python): create a UDP server to receive commands from a client;
- `bot_mover` node (python): publish messages on cmd\_vel topic based on commands received from the server.  


## Credits

Parts of code used for running the simulation in gazebo (directory `simulation_requirements`) were taken from [turtlebot3](http://wiki.ros.org/turtlebot3) library.

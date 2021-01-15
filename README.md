# turtlebot\_control

This repository contains the ROS-powered gazebo simulation for a Burger Turtlebot. Its extension (Android app) for a remote control of the Turtlebot's simulated movements can be found [here](https://github.com/emilia-szymanska/android_UDP_control).

The reaction to the app commands are listed below:
- _up_: robot moves forwards,
- _down_: robot moves backwards,
- _right_: robot rotates clockwise,
- _left_: robot rotates counterclockwise,
- _upright_: robot starts moving forwards in a circle (clockwise),
- _downright_: robot starts moving backwards in a circle (clockwise),
- _upleft_: robot starts moving forwards in a circle (counterclockwise),
- _downleft_: robot starts moving backwards in a circle (counterclockwise).
The message _center_ doesn't result in anything specific, it is open for further implementations and extensions.

## Installation

The repository was built for Ubuntu 20.04 with ROS Noetic and Python3.8.
Make sure you have ROS installed (follow the [guidelines](http://wiki.ros.org/noetic/Installation)).

Create a workspace, unless you have an existing one:
```
mkdir -p ~/workspace_name/src
cd ~/workspace_name/src
catkin_init_workspace
cd ~/workspace_name
catkin_make
```
To source the workspace, run (from the root of this repository): 
```
cd ~/workspace_name
source devel/setup.bash
```

Clone this package by running:
```
cd ~/workspace_name/src
git clone https://github.com/emilia-szymanska/turtlebot_control.git
```

The diagram shown below represents the architecture of the whole project.

![Image of Pipeline](https://github.com/emilia-szymanska/turtlebot_control/blob/master/pipeline_chart.png)


## Running the pipeline

To run the whole project, put the following commands in a terminal:
```
cd ~/workspace_name
roslaunch turtlebot3_control turtlebot3_app_control.launch
```

## Contents    

The package `turtlebot3_control` contains:

- `udp_server` node (python): create a UDP server to receive commands from a client;
- `bot_mover` node (python): publish messages on cmd\_vel topic based on commands received from the server.  

Both nodes have their parameters. For `udp_server` you can set an IP and a PORT.
Example:
```
rosrun turtlebot3_control udp_server _ip:="127.0.0.0"
``` 
As for a `bot_mover`, you can set:
- linear and angular scaling (with these values you set the constant values of both types of velocities to maximum\_velocity _x_ scale), 
- linear and angular circle scaling (these cause a similar action as above, but are valid only for a circular movement - when the bot receives commands such as upleft, upright, downleft, downright).

The parameters are set in the launch file, but also have some default values implemented.

## Credits

Parts of code used for running the simulation in gazebo (directory `simulation_requirements`) were taken from [turtlebot3](http://wiki.ros.org/turtlebot3) library.

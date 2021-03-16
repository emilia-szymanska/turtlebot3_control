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
- _center_: change to the "autonomous" ride mode.

Tha last option allows the user to specify the desired pose, which is then achieved by the robot on its own. 

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

Make sure that your workspace after applying major changes is built:
```
cd ~/workspace_name
catkin_make
```

The diagram shown below represents the architecture of the whole project.

![turtlebot3_control_background](https://user-images.githubusercontent.com/58346361/111316164-502ddf80-8663-11eb-90cd-2ef59fe28680.png)



## Running the pipeline

To run the whole project (for a real Turtlebot3), put the following commands in a terminal:
```
cd ~/workspace_name
roslaunch turtlebot3_control turtlebot3_app_control.launch
```
In case of a lack of a real Turtlebot3, replace the launch command with the following one to open a Gazebo simulation:
```
roslaunch turtlebot3_control turtlebot3_gazebo_control.launch
```

## Contents    

The package `turtlebot3_control` contains:

- `udp_server` node (Python3): creates a UDP server to receive commands from a client and sending back position feedback messages;
- `bot_mover` node (Python3): publishes messages on cmd\_vel topic based on commands received from the server;
- `desired_pose_commander` node (Python3): controls velocity commands based on the current and desired robot's position if the latter is specified by the user;
- `position_feedback` node (Python3): transforms the data from odometry messages to a user-friendly "x, y, theta" string data format.

Both nodes have their parameters. For `udp_server` you can set an IP and a PORT.
Example:
```
rosrun turtlebot3_control udp_server _ip:="127.0.0.0"
``` 
As for a `bot_mover`, you can set:
- linear and angular scaling (with these values you set the constant values of both types of velocities to maximum\_velocity _x_ scale), 
- linear and angular circle scaling (these cause a similar action as above, but are valid only for a circular movement - when the bot receives commands such as upleft, upright, downleft, downright).

The parameters are set in the launch file, but also have some default values implemented.

## Performance

The video presenting the perfomance of the package alongside with Android application in the [laboratory](https://lamor.fer.hr/) can be found [here](https://youtu.be/YFx6O14_POA).  

## Credits

Parts of code used for running the simulation in gazebo (directory `simulation_requirements`) were taken from [turtlebot3](http://wiki.ros.org/turtlebot3) library.

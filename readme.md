<h1 align=center> ROS Beginner Tutorial - Publisher / Subcriber </h1>
<p align="center">
<a href='https://opensource.org/licenses/MIT'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
</p>

## Project Overview

This is a basic publisher and subscriber in ROS for ENPM808X. It has two nodes viz.
* Talker (`src/talker.cpp`)
* Listener (`src/listener.cpp`)

It has a launch file `launchFile.launch` and a service `change_text` which changes the publisher message.

## Dependencies

These ROS nodes are made to be used on systems which have:
* ROS Kinetic
* Ubuntu 16.04

To install ROS, follow the instructions on this [link](http://wiki.ros.org/kinetic/Installation)

## How to build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/karanvivekbhargava/beginner_tutorials.git
cd ..
catkin_make
```

## How to run demo

1. After following the installation instructions above, you can either run all of them using roslaunch by typing the following command in the terminal. It will start both the talker and the listener nodes.
```
roslaunch beginner_tutorials launchFile.launch
```
You can optionally run the following to change the frequency of the publisher. In the following command replace 200 with any integer more than one.
```
roslaunch beginner_tutorials launchFile.launch freq:=200
```

2. If you'd like to run the nodes separately, then run roscore in the terminal as given below
```
roscore
```
To run talker, enter the command below in a new terminal window. You can enter any value more than or equal to one for the `<frequency>`
```
rosrun beginner_tutorials talker <frequency>
```
To run listener, enter the command below in a new terminal window.
```
rosrun beginner_tutorials listener
```

## Service

The talker (publisher) has a service to change the message.

If you'd like to run the service, type the following in a new terminal after starting roscore and talker from the methods above. It will change the publisher message to "sample text" in the example below.
```
rosservice call /change_text "sample text"
```

## Logging

To see the logger messages in the rqt_console GUI, run the command below after running the roscore, talker and listener nodes from the instructions above.

```
rosrun rqt_console rqt_console
```

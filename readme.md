<h1 align=center> ROS Beginner Tutorial - Publisher / Subcriber </h1>
<p align="center">
<a href='https://github.com/karanvivekbhargava/beginner_tutorials/blob/master/LICENSE'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
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

## Inspecting TF frames

The talker node broadcasts tf transforms to `\talk` relative to the `\world` frame. While roscore and talker are running we can use `tf_echo` to print the tf transforms.

```
rosrun tf tf_echo /world /talk
```

This gives us an output like the one below. The reference frame is given first and then the frame to be viewed.

```
At time 1510469826.773
- Translation: [0.935, -0.354, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.180, 0.984]
            in RPY (radian) [0.000, 0.000, -0.362]
            in RPY (degree) [0.000, 0.000, -20.723]
At time 1510469827.773
- Translation: [0.934, -0.357, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.181, 0.983]
            in RPY (radian) [0.000, 0.000, -0.365]
            in RPY (degree) [0.000, 0.000, -20.887]
At time 1510469828.773
- Translation: [0.933, -0.360, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.183, 0.983]
            in RPY (radian) [0.000, 0.000, -0.368]
            in RPY (degree) [0.000, 0.000, -21.085]
```

To generate a pdf connecting the reference frame to the other frame, type the command below

```
rosrun tf view_frames
```

Another fun way to view the frames would be through rviz, you can type

```
rosrun rviz rviz
```

Then select the world frame. Add a tf frame and you'd be able to see the frame orbiting around the origin.

## Running rostest

The unit tests have been written using gtest and rostest. To run the tests, you need to be in the catkin workspace parent folder. Then run the commands below

```
cd <path to catkin workspace>
catkin_make run_tests_beginner_tutorials
```

You can test using

```
rostest beginner_tutorials talkerTest.launch
```

The output should be similar to the one below.

```
testtalkerTest ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talkerTest/testServiceExists][passed]
[beginner_tutorials.rosunit-talkerTest/testChangeTestService][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/karan/.ros/log/rostest-karan-ubuntu-15698.log
```

## Recording bag files with the launch file

You must first build the project using catkin_make as described earlier. You may run the command below to launch the nodes and record all the topics. The bag file will be in the results directory once the recording is complete.

```
roslaunch beginner_tutorials launchFile.launch record:=true
```

## Playing back the bag file with the Listener node demonstration

First, navigate to the results folder.

```
cd <path to repository>/results
```

To inspect the bag file, ensure that the roscore and listener nodes are running. Then in a new terminal, enter the command below.

```
rosbag play listener.bag
```

You will be able to see the listener node output on the screen. It would be playing the same messages that were recorded.

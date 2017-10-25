<h1 align=center> Beginner Tutorials - Publisher / Subcriber </h1>
<p align="center">
<a href="https://travis-ci.org/karanvivekbhargava/beginner_tutorials">
<img src="https://travis-ci.org/karanvivekbhargava/beginner_tutorials.svg?branch=master">
</a>
<a href='https://coveralls.io/github/karanvivekbhargava/beginner_tutorials?branch=master'><img src='https://coveralls.io/repos/github/karanvivekbhargava/beginner_tutorials/badge.svg?branch=master'/></a>
<a href='https://opensource.org/licenses/MIT'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
</p>

## Project Overview

This is a basic publisher and subscriber in ROS.

## Feature List
* Talker
* Listener
---

## Dependencies

The butler software stack has the following dependencies:
* cmake
* googletest
* opencv

To install opencv, follow the instructions on [link](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html)

## How to build - standard install via command-line
```
git clone --recursive https://github.com/karanvivekbhargava/robot-butler-enpm808x
cd <path to repository>
mkdir build
cd build
cmake ..
make
```

## How to run demo

After following the installation instructions above, you can try three different images from the data folder by specifying the argument with the program

To run for image_left.jpg, kindly enter the function below
```
./app/shell-app left
```
To run for image_center.jpg, kindly enter the function below
```
./app/shell-app center
```
To run for image_right.jpg, kindly enter the function below
```
./app/shell-app right
```

## How to run tests

After following the building instructions, run the command below
```
./test/cpp-test
```

## How to generate documentation

Although the repository contains the documentation, if you'd still like to generate it then follow the instructions below.

```
sudo apt-get install doxygen
sudo apt-get install doxywizard
doxywizard
```

Once doxywizard is open, select the workspace as the repository. Fill in the details as required and set the source code folder to the repository as well. Create a new folder in the repository and select that as the destination directory. Proceed with the default settings and generate the documentation.

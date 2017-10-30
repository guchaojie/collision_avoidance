
# Intelligent Collision Avoidance


This repo is the place holder for enhanced collision avoidance solution based on ROS system (highly 
depend on navigation stack). The solution adopts and enables new sensor inputs (such as range 
sensor, RGB-D camera sensor, etc.) to extends the ability of common navigation by such points as:

- multi-sensor fusion depend on navigation plugin mechanism.
- different collision avoidance policy, accellerated by deep learning and object tracking.

## Directory Structure

### <i class="icon-upload"></i> turtlebot_navigation
This folder holds navigation customization for turtlebot2 robot base, as well our pre-analysis 
result for POC robot base (say, water base) temporally.

### <i class="icon-upload"></i> water_navigation (in design)
This folder holds navigation customization for [water](www.yunji.com) robot base, as well our 
pre-analysis result for POC robot base (say, water base) temporally.


### <i class="icon-upload"></i> sonar_filter
This folder holds the simple linear filtering algorithm implementation. Currently, the filtering 
algorithm addresses the noise of sonar input, and filters them out.

## Build Steps
This package was verified under Ubuntu 16.04 with ROS kinectic environment.

### 1. Make sure ROS dependencies and catkin tools ready for use
Follow [ROS Wiki](http://wiki.ros.org/ROS/Installation) and [catkin 
tool](http://wiki.ros.org/catkin) to download related packages in your Linux machine.

> NOTE: Currently for AMR POC Demo, turtlebot_navigation package is ignored for catkin_make. 
> If you want to use turtlebot base for debugging or demo use, please remove CATKIN_IGNORE under
> its folder before code building first.

### 2. Download code to your local storage
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.intel.com/otc-rse/AMR_ros_collision_avoidance.git
	cd AMR_ros_collision_avoidance/navigation_layers
	git submodule init
	git submodule update
	git checkout demo

### 3. Build code
    cd ~/catkin_ws
    catkin_make

### 4. Sourcing the ROS workspace
After building the workspace, source it via:
    source devel/setup.bash

## Security

For security issues, please send mail to wei.zhi.liu@intel.com

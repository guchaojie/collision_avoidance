
## navigation configration for Water Robot


This repo is the place holder for configuration files, parameter files and launch files of navigation stack.


### Build Steps
This package was verified under Ubuntu 16.04 with ROS kinectic environment.

#### 1. Make sure ROS dependencies and catkin tools ready for use
Follow [ROS Wiki](http://wiki.ros.org/ROS/Installation) and [catkin
tool](http://wiki.ros.org/catkin) to download related packages in your Linux machine.

#### 2. Download code to your local storage
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.intel.com/otc-rse/AMR_ros_collision_avoidance.git

#### 3. Build code
    cd ~/catkin_ws
    catkin_make

#### 4. Sourcing the ROS workspace
After building the workspace, source it via:
    source devel/setup.bash

### Launch

#### 1. Launch navigation with gmapping
    roslaunch water_navigation ros_gmapping_slave.launch

#### 2. Launch navigation with AMCL
    roslaunch water_navigation amcl_demo.launch


### Security

   > For security issues, please send mail to wei.zhi.liu@intel.com

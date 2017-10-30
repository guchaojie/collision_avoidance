cmake_minimum_required(VERSION 2.8.3)
project(ca_policy)

add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs
		object_pipeline_msgs
		object_bridge_msgs
		ros_yolo_msgs
		ca_policy_msgs)

# What other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)


###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

# Add_executables
add_executable(ca_policy
  src/object_merger_node.cpp
  src/ca_policy.cpp
  src/ca_policy_manager.cpp
  src/object_frame.cpp
  src/object_merger.cpp
  src/obstacles.cpp
  )
target_link_libraries(ca_policy ${catkin_LIBRARIES})


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ca_policy
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


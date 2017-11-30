cmake_minimum_required(VERSION 2.8.3)
project(ca_policy)

add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs
		object_analytics_msgs
		object_bridge_msgs
		object_msgs
		ca_policy_msgs
		message_filters
		)

## dynamic reconfigure config
#generate_dynamic_reconfigure_options(cfg/CaObjectFrame.cfg)

# What other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs object_analytics_msgs object_msgs message_filters
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

# Add_executables
add_executable(vision_object
  src/object_merger_node.cpp
  src/object_frame.cpp
  src/object_merger.cpp
  src/obstacles.cpp
  )
target_link_libraries(vision_object ${catkin_LIBRARIES})
add_dependencies(vision_object ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ca_policy
  src/ca_policy_node.cpp
  src/ca_policy_ros.cpp
  src/ca_policy.cpp
  src/ca_policy_normal.cpp
  src/ca_policy_social.cpp
  src/ca_policy_manager.cpp
  )
target_link_libraries(ca_policy ${catkin_LIBRARIES})
#add_dependencies(ca_policy ${PROJECT_NAME}_gencfg)
add_dependencies(ca_policy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS vision_object
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ca_policy
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



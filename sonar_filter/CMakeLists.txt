cmake_minimum_required(VERSION 2.8.3)
project(sonar_filter)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS tf roscpp sensor_msgs)

# What other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS tf roscpp sensor_msgs
)


###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

# Add_executables
add_executable(sonar_filter src/sonar_filter.cpp)
target_link_libraries(sonar_filter ${catkin_LIBRARIES})


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS sonar_filter
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


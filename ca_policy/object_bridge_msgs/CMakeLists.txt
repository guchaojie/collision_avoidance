cmake_minimum_required(VERSION 2.8.3)
project(object_bridge_msgs)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs sensor_msgs geometry_msgs)



add_message_files(
   DIRECTORY msg
   FILES
   SocialObject.msg
   ObjectMerged.msg
   ObjectMergedStamped.msg
   ObjectsInFrameMerged.msg
)

generate_messages(
   DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# What other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS std_msgs message_runtime geometry_msgs
)

cmake_minimum_required(VERSION 2.8.3)
project(object_bridge_msgs)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs 
object_pipeline_msgs)



add_message_files(
   DIRECTORY msg
   FILES
   ObjectMerged.msg
   ObjectMergedStamped.msg
   ObjectsInFrameMerged.msg
)

generate_messages(
   DEPENDENCIES geometry_msgs std_msgs object_pipeline_msgs
)

# What other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS geometry_msgs std_msgs object_pipeline_msgs
)

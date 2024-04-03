cmake_minimum_required(VERSION 3.0.2)
project(acsense_ros_interfaces)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        rospy
        std_msgs
        message_generation
        )

add_message_files(
        FILES
        AcSenseRawData.msg
        AcSenseRawDataSingleChannel.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        CATKIN_DEPENDS roscpp rospy
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

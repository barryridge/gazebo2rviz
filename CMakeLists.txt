cmake_minimum_required(VERSION 2.8.3)
project(gazebo2rviz)

find_package(catkin REQUIRED COMPONENTS
  rospy
  pysdf
  std_msgs
  shape_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  MeshStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  shape_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo2rviz
  CATKIN_DEPENDS rospy pysdf
  #  DEPENDS python-rospkg
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
install(PROGRAMS
  scripts/gazebo2tf_node.py
  scripts/gazebo2marker_node.py
  scripts/sdf2marker_node.py
  scripts/sdf2tfstatic_node.py
  scripts/sdf2extract_tfstatic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

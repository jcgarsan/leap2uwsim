cmake_minimum_required(VERSION 2.8.3)
project(leap2uwsim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
#  sensor_msgs
)

# find_package(uwsim REQUIRED)
# find_package(leap_tracker REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
#	leap2uwsim
  CATKIN_DEPENDS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	nav_msgs
	robot_state_publisher
	sensor_msgs
#  DEPENDS
#	uwsim
#	leap_tracker
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(leap2uwsim
#   src/${PROJECT_NAME}/leap2uwsim.cpp
#)

## Declare a cpp executable
add_executable(leap2uwsim src/leap2uwsim.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(beginner_tutorials_node beginner_tutorials_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(leap2uwsim
	${catkin_LIBRARIES}
)


#############
## Install ##
#############
install(
	TARGETS
		leap2uwsim
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(DIRECTORY launch/
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)

#install(DIRECTORY include/${PROJECT_NAME}
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#install(DIRECTORY src/
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   FILES_MATCHING PATTERN "leap2uwsim" 
#   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
#               GROUP_READ GROUP_EXECUTE
#               WORLD_READ WORLD_EXECUTE
#)


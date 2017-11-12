cmake_minimum_required(VERSION 2.8.3)
project(descartes_parser)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	descartes_planner
	descartes_msgs
)

catkin_package(
	INCLUDE_DIRS
	include
	LIBRARIES
	${PROJECT_NAME}
	CATKIN_DEPENDS
	roscpp
	descartes_planner
	descartes_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/descartes_parser.cpp
	)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)
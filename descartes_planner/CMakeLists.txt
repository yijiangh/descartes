cmake_minimum_required(VERSION 2.8.11)
project(descartes_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_trajectory
  moveit_core
  roscpp
  pluginlib
  cmake_modules
  rviz_visual_tools
  eigen_conversions
  moveit_msgs
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    test
  LIBRARIES
    descartes_planner
  CATKIN_DEPENDS
    roscpp
    moveit_core
    descartes_core
    descartes_trajectory
    eigen_conversions
    moveit_msgs
  DEPENDS
    Boost
    EIGEN3
)

###########
## Build ##
###########
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
)

## DescartesTrajectoryPt lib
add_library(descartes_planner
            src/planning_graph.cpp
            src/sparse_planner.cpp
            src/dense_planner.cpp
            src/plugin_init.cpp
            src/ladder_graph_dag_search.cpp
            src/ladder_graph_dag_search_lazy_collision.cpp
#            src/graph_builder.cpp
#            src/capsulated_ladder_tree_RRTstar.cpp
)

target_link_libraries(descartes_planner
                      ${catkin_LIBRARIES}
)


#############
## Install ##
#############
install(
    TARGETS ${PROJECT_NAME} 
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)

  set(UTEST_PLANNER_SRC_FILES 
      test/utest.cpp
      test/dense_planner.cpp
      test/sparse_planner.cpp
      test/planning_graph_tests.cpp
      test/utils/trajectory_maker.cpp)
  catkin_add_gtest(${PROJECT_NAME}_planner_utest ${UTEST_PLANNER_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_planner_utest descartes_planner)

endif()

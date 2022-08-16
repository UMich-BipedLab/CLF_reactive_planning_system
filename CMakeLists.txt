cmake_minimum_required(VERSION 2.8.3)
project(cassie_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")

# warnings
# if ( CMAKE_COMPILER_IS_GNUCC )
#     set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
# endif()
# if ( MSVC )
#     set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")
# endif()
# add_definitions(-w)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  visualization_msgs
  message_filters
  #velodyne_pointcloud
  roslib
  grid_map_ros  
  grid_map_core
  inekf_msgs
  planner_msgs
  grid_map_msgs
  rviz_visual_tools
)

find_package(PCL 1.2 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   INCLUDE_DIRS external
   LIBRARIES ${PROJECT_NAME}
   DEPENDS
Eigen3
  grid_map_ros  
  grid_map_core
  CATKIN_DEPENDS geometry_msgs

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  external/cereal/include
  /usr/local/include/
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)
include_directories(/usr/local/lib)
include_directories(/usr/local/include/)

# message(STATUS "--------------------------------------------------------------{")
# get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
# message("inc_dirs = ${inc_dirs}")
# message("catkin = ${catkin_LIBRARIES}")
# message(STATUS "${INCLUDE_DIRECTORIES}")
# message(STATUS "--------------------------------------------------------------{")
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


####################################################################################
#### Simple Testing
####################################################################################
add_executable(test_plane_fitting src/test_plane_fitting.cpp src/fake_map.cpp
    src/utils/plane.cpp
    src/multivariate_gaussian.cpp src/clf_rrt.cpp src/cassie_rrt_tree.cpp src/lyapunov_distance.cpp
    src/local_chart.cpp src/standalone_lyapunov_distance.cpp src/local_map.cpp
    src/map_operation.cpp
    src/sample_pose.cpp src/pose.cpp 
    src/lyapunov_path.cpp 
    src/standalone_local_chart.cpp
    src/standalone_omni_local_chart.cpp
    src/standalone_omni_lyapunov_distance.cpp
    src/omni_local_chart.cpp
    src/utils/plotting.cpp
    src/map_cost.cpp
    src/communication.cpp
    src/planner_info_to_controller_t.c
    src/controller_info_to_planner_t.c
    src/driver.cpp
    src/control_commands.cpp
    src/map_operation.cpp
    src/lie_group.cpp
)
add_dependencies(test_plane_fitting ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_plane_fitting ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)


# add_executable(simple_test src/simple_test.cpp src/fake_map.cpp
#     src/multivariate_gaussian.cpp  src/local_map.cpp
#     src/map_operation.cpp
# )
# add_dependencies(simple_test ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS})
# target_link_libraries(simple_test ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )

# add_executable(test_udp src/test_udp.cpp src/fake_map.cpp
#     src/multivariate_gaussian.cpp src/clf_rrt.cpp src/cassie_rrt_tree.cpp src/lyapunov_distance.cpp
#     src/local_chart.cpp src/standalone_lyapunov_distance.cpp src/local_map.cpp
#     src/map_operation.cpp
#     src/sample_pose.cpp src/pose.cpp 
#     src/lyapunov_path.cpp 
#     src/standalone_local_chart.cpp
#     src/standalone_omni_local_chart.cpp
#     src/standalone_omni_lyapunov_distance.cpp
#     src/omni_local_chart.cpp
#     src/utils/plotting.cpp
#     src/map_cost.cpp
#     src/communication.cpp
#     src/planner_info_to_controller_t.c
#     src/controller_info_to_planner_t.c
#     src/control_commands.cpp
# )
# add_dependencies(test_udp ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_udp ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )
 
# add_executable(test_rviz_tool src/test_rviz_tool.cpp src/fake_map.cpp
#     src/multivariate_gaussian.cpp src/clf_rrt.cpp src/cassie_rrt_tree.cpp src/lyapunov_distance.cpp
#     src/local_chart.cpp src/standalone_lyapunov_distance.cpp src/local_map.cpp
#     src/map_operation.cpp
#     src/sample_pose.cpp src/pose.cpp 
#     src/lyapunov_path.cpp 
#     src/standalone_local_chart.cpp
#     src/standalone_omni_local_chart.cpp
#     src/standalone_omni_lyapunov_distance.cpp
#     src/omni_local_chart.cpp
#     src/utils/plotting.cpp
#     src/map_cost.cpp
#     src/communication.cpp
#     src/planner_info_to_controller_t.c
#     src/controller_info_to_planner_t.c
#     src/driver.cpp
#     src/control_commands.cpp
#     src/lie_group.cpp
# )
# add_dependencies(test_rviz_tool ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_rviz_tool ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )
 
# add_executable(unit_test src/unit_test.cpp src/fake_map.cpp
#     src/multivariate_gaussian.cpp src/clf_rrt.cpp src/cassie_rrt_tree.cpp src/lyapunov_distance.cpp
#     src/local_chart.cpp src/standalone_lyapunov_distance.cpp src/local_map.cpp
#     src/map_operation.cpp
#     src/sample_pose.cpp src/pose.cpp 
#     src/lyapunov_path.cpp 
#     src/standalone_local_chart.cpp
#     src/standalone_omni_local_chart.cpp
#     src/standalone_omni_lyapunov_distance.cpp
#     src/omni_local_chart.cpp
#     src/utils/plotting.cpp
#     src/map_cost.cpp
#     src/communication.cpp
#     src/planner_info_to_controller_t.c
#     src/controller_info_to_planner_t.c
#     src/driver.cpp
#     src/control_commands.cpp
#     src/lie_group.cpp
# )
# add_dependencies(unit_test ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS})
# target_link_libraries(unit_test ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )
# ####################################################################################
# 
# 
# 
# ####################################################################################
# #### publisher
# ####################################################################################
# add_executable(fake_map_publisher src/map_publisher.cpp src/fake_map.cpp
#     src/multivariate_gaussian.cpp 
#     src/map_operation.cpp
# )
# add_dependencies(fake_map_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS})
# target_link_libraries(fake_map_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )
# 
# 
# add_executable(static_robot_publisher src/static_robot_publisher.cpp
#     src/utils/plotting.cpp
# )
# add_dependencies(static_robot_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS})
# target_link_libraries(static_robot_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )
# 
# 
add_executable(fake_robot_publisher src/fake_robot_publisher.cpp
    src/fake_robot.cpp
    src/utils/plotting.cpp
)
add_dependencies(fake_robot_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_robot_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
# 
# 
# ####################################################################################
# 
# 
# 
# ####################################################################################
# #### RRT TESTING 
# ####################################################################################
# add_executable(test_rrt src/test_rrt.cpp src/fake_map.cpp
#     src/multivariate_gaussian.cpp 
#     src/clf_rrt.cpp src/cassie_rrt_tree.cpp 
#     src/lyapunov_distance.cpp
#     src/local_chart.cpp src/standalone_lyapunov_distance.cpp src/local_map.cpp
#     src/map_operation.cpp
#     src/sample_pose.cpp src/pose.cpp 
#     src/lyapunov_path.cpp 
#     src/standalone_local_chart.cpp
#     src/standalone_omni_local_chart.cpp
#     src/standalone_omni_lyapunov_distance.cpp
#     src/omni_local_chart.cpp
#     src/utils/plotting.cpp
#     src/map_cost.cpp
#     src/control_commands.cpp
# )
# add_dependencies(test_rrt ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_rrt ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )
# ####################################################################################





####################################################################################
#### PIPELINE TESTING 
####################################################################################
add_executable(test_pipeline src/main.cpp src/fake_map.cpp
    src/utils/plane.cpp
    src/multivariate_gaussian.cpp src/clf_rrt.cpp src/cassie_rrt_tree.cpp src/lyapunov_distance.cpp
    src/local_chart.cpp src/standalone_lyapunov_distance.cpp src/local_map.cpp
    src/map_operation.cpp
    src/sample_pose.cpp src/pose.cpp 
    src/lyapunov_path.cpp 
    src/standalone_local_chart.cpp
    src/standalone_omni_local_chart.cpp
    src/standalone_omni_lyapunov_distance.cpp
    src/omni_local_chart.cpp
    src/utils/plotting.cpp
    src/map_cost.cpp
    src/communication.cpp
    src/planner_info_to_controller_t.c
    src/controller_info_to_planner_t.c
    src/driver.cpp
    src/control_commands.cpp
    src/map_operation.cpp
    src/lie_group.cpp
)
add_dependencies(test_pipeline ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pipeline ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
####################################################################################



# target_link_libraries(wavefield_test ${catkin_LIBRARIES} /usr/local/lib/libgp.a)
# add_executable(cassie_planner src/main.cc src/Dstar.cc src/waypoint_data_t.c src/wavefront_frontier_detection.cc)
# target_link_libraries(cassie_planner ${catkin_LIBRARIES})
# add_dependencies(cassie_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

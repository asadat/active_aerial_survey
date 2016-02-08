cmake_minimum_required(VERSION 2.8.3)

project(active_survey)

FIND_PACKAGE(Eigen3 3.0.1 REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 -o3 ${CMAKE_CXX_FLAGS} -g -Wall")

set(aas_LIBRARIES GL GLU glut gp)

set(aas_SOURCES
    src/active_survey.cpp
    src/active_survey_node.cpp
    src/active_survey_param.cpp
    src/environment_model/environment_model.cpp
    src/environment_model/grid_cell.cpp
    src/environment_model/grid.cpp
    src/environment_model/gaussian_field.cpp
    src/mav/sensor.cpp
    src/mav/mav.cpp
    src/mav/mav_controller.cpp
    src/planner/behaviour_controller.cpp
    src/planner/behaviour_planner.cpp
    src/planner/waypoint.cpp
    src/planner/plan.cpp
    src/math/polygon.cpp
    include/math/polygon.h
    include/planner/trajectory_planner.h
    include/planner/plan.h
    include/planner/waypoint.h
    include/planner/behaviour_controller.h
    include/planner/behaviour_planner.h
    include/mav/mav_controller.h
    include/mav/mav.h
    include/mav/sensor.h
    include/environment_model/random_environment_generator.h
    include/environment_model/gaussian_field.h
    include/environment_model/grid_cell.h
    include/environment_model/grid.h
    include/environment_model/environment_model.h
    include/active_survey.h    
    include/active_survey_param.h
    include/utility.h
)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  nav_msgs
  message_generation
  image_transport
  cv_bridge
  pelican_ctrl
  )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy  geometry_msgs  std_msgs  nav_msgs  message_generation  image_transport  cv_bridge  pelican_ctrl  )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${LIBGP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})


  
add_executable(active_survey ${aas_SOURCES})
target_link_libraries(active_survey  ${aas_LIBRARIES} ${catkin_LIBRARIES} ${LIBGP_LIBRARIES})


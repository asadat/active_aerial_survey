cmake_minimum_required(VERSION 2.8.3)

project(active_survey)

FIND_PACKAGE(LIBGP)
FIND_PACKAGE(Eigen3 3.0.1 REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11  ${CMAKE_CXX_FLAGS} -g -Wall")

set(aas_LIBRARIES GL GLU glut gp)

set(aas_SOURCES
    src/active_survey.cpp
    src/active_survey_node.cpp
    src/active_survey_param.cpp
    src/environment_model.cpp
    src/grid_cell.cpp
    src/grid.cpp
    src/gaussian_field.cpp
    include/random_environment_generator.h
    include/gaussian_field.h
    include/grid_cell.h
    include/grid.h
    include/environment_model.h
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


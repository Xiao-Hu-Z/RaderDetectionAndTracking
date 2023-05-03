cmake_minimum_required(VERSION 2.8.3)
project(imm_ukf_pda_track)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  pcl_ros
  roscpp
  roslint
  tf
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#imm_ukf_pda
add_executable(imm_ukf_pda
  imm_ukf_pda_main.cpp
  imm_ukf_pda.cpp
  ukf.cpp
)

target_link_libraries(imm_ukf_pda
  ${catkin_LIBRARIES}
)
add_dependencies(imm_ukf_pda
  ${catkin_EXPORTED_TARGETS}
)


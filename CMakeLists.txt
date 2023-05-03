cmake_minimum_required(VERSION 2.8.3)
project(rader_detection_tracking)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(roscpp REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 添加子目录并构建
add_subdirectory(lib/pre_process)
add_subdirectory(lib/bounding_box)
add_subdirectory(lib/visualization)

set(CONFIG_FILE_PATH "${PROJECT_SOURCE_DIR}/config/config_manager.yaml")
set(ROOT_PATH "${PROJECT_SOURCE_DIR}")

set(rader_demo_config_h_in "${CMAKE_SOURCE_DIR}/lib/common/configure/rader_demo_config.h.in")
set(rader_demo_config_h "${CMAKE_SOURCE_DIR}/lib/common/configure//rader_demo_config.h")
configure_file("${rader_demo_config_h_in}" "${rader_demo_config_h}")

add_executable(${PROJECT_NAME} rader_detection_tracking.cpp)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${PCL_LIBRARIES}
 )



add_executable(${PROJECT_NAME}_bin rader_detection_tracking_bin.cpp)
target_link_libraries(${PROJECT_NAME}_bin
  ${OpenCV_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${PCL_LIBRARIES}
 )

#  add_executable(${PROJECT_NAME}_bag rader_detection_tracking_bag.cpp)
#  target_link_libraries(${PROJECT_NAME}_bag
#    ${OpenCV_LIBRARIES}
#    ${roscpp_LIBRARIES}
#    ${PCL_LIBRARIES}
#   )



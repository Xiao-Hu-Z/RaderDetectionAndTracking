
cmake_minimum_required(VERSION 3.10)
project(dbscan)

# add_compile_options(-std=c++11)
# find_package(yaml-cpp REQUIRED)

find_package(roscpp REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)


add_library(${PROJECT_NAME} SHARED dbscan.cpp)

add_executable(${PROJECT_NAME} SHARED dbscan.cpp dbscan_test.cpp)

target_link_libraries(${PROJECT_NAME} 
  ${roscpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  )

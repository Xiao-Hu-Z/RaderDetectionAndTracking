cmake_minimum_required(VERSION 3.10)
project(pre_process)

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

set(FILE_DIR "")
list(APPEND FILE_DIR src)
list(APPEND FILE_DIR include)

set(ALL_SRCS "")
foreach(dir ${FILE_DIR})
    file(GLOB_RECURSE TEMP_SRCS "${dir}/*.cpp" "${dir}/*.hpp" "${dir}/*.h")
    list(APPEND ALL_SRCS ${TEMP_SRCS})
endforeach()

add_library(${PROJECT_NAME} SHARED ${ALL_SRCS})

target_link_libraries(${PROJECT_NAME} 
  ${roscpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  )


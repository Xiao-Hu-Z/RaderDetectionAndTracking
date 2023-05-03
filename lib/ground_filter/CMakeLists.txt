cmake_minimum_required(VERSION 3.5)
project(ground_filter)

set(FILE_DIR "")
list(APPEND FILE_DIR src)
list(APPEND FILE_DIR include/ground_filter)

set(ALL_SRCS "")
foreach(dir ${FILE_DIR})
    file(GLOB_RECURSE TEMP_SRCS "${dir}/*.cpp" "${dir}/*.hpp" "${dir}/*.h")
    list(APPEND ALL_SRCS ${TEMP_SRCS})
endforeach()

add_library(${PROJECT_NAME} SHARED ${ALL_SRCS})
target_include_directories(${PROJECT_NAME} 
                           PUBLIC 
                           ${INCLUDE_DIRS}
                           ${ROS_DEPENDECE_INCLUDE}
                           ${THIRD_PARTY_INCLUDE_DIR}
                           )
target_link_libraries(${PROJECT_NAME} 
                      ${LIDAR_DEMO_LIBRARIES}
                      ${ALL_THIRD_PARTY_LIBS}
                      ${ROS_DEPENDECE_LIBS})

set(ground_filter ${PROJECT_NAME} PARENT_SCOPE)
set(GROUND_FILTER_SRCS ${ALL_SRCS} PARENT_SCOPE)
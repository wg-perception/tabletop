cmake_minimum_required(VERSION 2.8)
project(tabletop)
include(cmake/pcl-search.cmake)

#find_package(catkin)
#find_package(genmsg)

#catkin_project(tabletop
#               INCLUDE_DIRS include)

#add_message_files(
#  DIRECTORY msg
#  FILES Table.msg)

#generate_messages()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${PCL_INCLUDE_DIRS})

add_subdirectory(src)

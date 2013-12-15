cmake_minimum_required(VERSION 2.8)
project(object_recognition_tabletop)

find_package(catkin REQUIRED ecto household_objects_database
                             moveit_core object_recognition_core object_recognition_msgs opencv_candidate
                             pluginlib rosconsole roscpp shape_msgs sql_database tf)
catkin_package(INCLUDE_DIRS include)

# deal with Python
catkin_python_setup()

add_subdirectory(python)

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE 1)

# include some directories
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}
                    include
)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})

add_subdirectory(src)

# add the tests
add_subdirectory(test)

# add the docs
catkin_doxygen(object_recognition_tabletop-doxygen ${CMAKE_CURRENT_SOURCE_DIR}/include)
ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc)

# install the configuration files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/conf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install the include folder
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/object_recognition_tabletop/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install the plugin file
install(FILES ${PROJECT_SOURCE_DIR}/db_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

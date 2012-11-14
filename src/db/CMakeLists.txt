find_package(ecto REQUIRED)

# Create a library for tabletop db interfaces
add_library(tabletop_db SHARED
            db_sql_household.cpp
            db_mesh.cpp
)

if (ROS_FUERTE_FOUND)
    include_directories(SYSTEM
                        /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/
                        /opt/ros/fuerte/stacks/object_manipulation/household_objects_database/include/
                        /opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include/
                        /opt/ros/fuerte/stacks/sql_database/database_interface/include/
    )
    find_library(household_objects_database_LIBRARIES household_objects_database /opt/ros/fuerte/stacks/object_manipulation/household_objects_database/lib)
else()
    include_directories(SYSTEM
                        /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/
                        /opt/ros/fuerte/stacks/object_manipulation/household_objects_database/include/
                        /opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include/
                        /opt/ros/fuerte/stacks/sql_database/database_interface/include/
    )
    find_library(household_objects_database_LIBRARIES household_objects_database /opt/ros/fuerte/stacks/object_manipulation/household_objects_database/lib)
endif()

target_link_libraries(tabletop_db
                      ${object_recognition_core_LIBRARIES}
                      ${household_objects_database_LIBRARIES}
)

if (ROS_FUERTE_FOUND)
install(TARGETS tabletop_db
        DESTINATION lib
)
else()
install(TARGETS tabletop_db
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
endif()


# create a Python module to wrap the DB functionalities
find_package(PythonLibs)
include_directories(SYSTEM ${PYTHON_INCLUDE_PATH})
add_library(tabletop_db_interface SHARED
            module_python.cpp
            wrap_object_db.cpp
)

target_link_libraries(tabletop_db_interface
                      tabletop_db
)

if (ROS_FUERTE_FOUND)
set(ECTO_PYTHON_BUILD_PATH ${CMAKE_BINARY_DIR}/gen/py/)
set_target_properties(tabletop_db_interface PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${ECTO_PYTHON_BUILD_PATH}/${PROJECT_NAME}
                      OUTPUT_NAME db_interface
                      PREFIX ""
)

install(TARGETS tabletop_db_interface
        DESTINATION ${ECTO_PYTHON_INSTALL_PATH}/${PROJECT_NAME}
)
else()
set_target_properties(tabletop_db_interface PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
                      OUTPUT_NAME db_interface
                      PREFIX ""
)
 
install(TARGETS tabletop_db_interface
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
endif()

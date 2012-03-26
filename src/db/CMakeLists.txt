# Create a library for tabletop db interfaces
add_library(tabletop_db SHARED
            db_sql_household.cpp
            db_mesh.cpp
)

find_package(object_recognition_core REQUIRED)

include_directories(SYSTEM
                    ${object_recognition_core_INCLUDE_DIRS}
)

if (ROS_ELECTRIC_FOUND)
    find_ros_package(arm_navigation_msgs)
    find_ros_package(household_objects_database_msgs)
    find_ros_package(object_manipulation)
    find_ros_package(sql_database)
    find_ros_package(household_objects_database)
    include_directories(SYSTEM
                        ${arm_navigation_msgs_INCLUDE_DIRS}
                        ${household_objects_database_msgs_INCLUDE_DIRS}
                        ${object_manipulation_INCLUDE_DIRS}
                        ${sql_database_INCLUDE_DIRS}
    )
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
                      object_recognition_core_db
                      ${household_objects_database_LIBRARIES}
)

install(TARGETS tabletop_db
        DESTINATION lib
)


# create a Python module to wrap the DB functionalities
add_library(tabletop_db_interface SHARED
            module_python.cpp
            wrap_object_db.cpp
)

target_link_libraries(tabletop_db_interface
                      tabletop_db
)

set_target_properties(tabletop_db_interface PROPERTIES
                      OUTPUT_NAME db_interface
                      PREFIX ""
)

install(TARGETS tabletop_db_interface
        DESTINATION ${PYTHON_PACKAGES_PATH}/tabletop/
)

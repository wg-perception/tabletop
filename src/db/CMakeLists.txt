# Create a library for tabletop db interfaces
add_library(tabletop_db SHARED
            db_sql_household.cpp
            db_mesh.cpp
)

find_package(object_recognition_core REQUIRED)

include_directories(SYSTEM
                    ${object_recognition_core_INCLUDE_DIRS}
                    /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/
)

target_link_libraries(tabletop_db
                      object_recognition_core_db
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
                      object_recognition_core_db
)

set_target_properties(tabletop_db_interface PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${python_LIBRARY_OUTPUT_DIRECTORY}/
                      OUTPUT_NAME interface
                      PREFIX ""
)

install(TARGETS tabletop_db_interface
        DESTINATION ${PYTHON_PACKAGES_PATH}/tabletop/
)

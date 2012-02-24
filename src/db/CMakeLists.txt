# Create a library for tabletop db interfaces
add_library(tabletop_db SHARED
            db_sql_household.cpp
            db_mesh.cpp
)

find_package(object_recognition REQUIRED)

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

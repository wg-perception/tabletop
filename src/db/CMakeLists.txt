# Create a library for tabletop db interfaces
add_library(tabletop_db db_sql_household.cpp
                        db_mesh.cpp
)

target_link_libraries(tabletop_db ${catkin_LIBRARIES})

install(TARGETS tabletop_db
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# create a Python module to wrap the DB functionalities
find_package(PythonLibs)
include_directories(SYSTEM ${PYTHON_INCLUDE_PATH})
add_library(tabletop_db_interface module_python.cpp
                                  wrap_object_db.cpp
)
target_link_libraries(tabletop_db_interface tabletop_db ${catkin_LIBRARIES})

set_target_properties(tabletop_db_interface PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
                      OUTPUT_NAME db_interface
                      PREFIX ""
)

install(TARGETS tabletop_db_interface
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

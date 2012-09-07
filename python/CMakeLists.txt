if (ROS_GROOVY_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py.plain.in
               ${CATKIN_BUILD_PREFIX}/${CATKIN_PROJECT_PYTHON_DESTINATION}/ecto_cells/__init__.py
               @ONLY
)

install(FILES ${CATKIN_BUILD_PREFIX}/${CATKIN_PROJECT_PYTHON_DESTINATION}/ecto_cells/__init__.py
        DESTINATION ${CATKIN_PROJECT_PYTHON_DESTINATION}/ecto_cells/__init__.py
)
else()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py.plain.in
               ${ECTO_PYTHON_BUILD_PATH}/ecto_cells/__init__.py
               @ONLY
)

install(FILES ${ECTO_PYTHON_BUILD_PATH}/ecto_cells//__init__.py
        DESTINATION ${ECTO_PYTHON_INSTALL_PATH}/tabletop/ecto_cells
)
endif()

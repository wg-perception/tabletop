#catkin_python_setup()

#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tabletop/__init__.py
#               ${python_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
#               @ONLY
#)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py.plain.in
               ${CMAKE_BINARY_DIR}/gen/py/tabletop/ecto_cells/__init__.py
               @ONLY
)

install(FILES ${CMAKE_BINARY_DIR}/gen/py/tabletop/ecto_cells/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/tabletop/ecto_cells
)

#install targets for all things python
#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tabletop
#        DESTINATION ${PYTHON_PACKAGES_PATH}
#        COMPONENT main
#        FILES_MATCHING PATTERN "*.py"
#)

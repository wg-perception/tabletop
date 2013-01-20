ectomodule(ecto_tabletop DESTINATION ${PROJECT_NAME}/ecto_cells
                         INSTALL
  ecto_tabletop.cpp
  wrap_tabletop_TableArray.cpp
)

link_ecto(ecto_tabletop ${catkin_LIBRARIES}
                        ${OpenCV_LIBRARIES}
)

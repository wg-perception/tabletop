find_package(ecto_ros REQUIRED)
include_directories(SYSTEM ${ecto_ros_INCLUDE_DIRS})

ectomodule(ecto_tabletop DESTINATION ${PROJECT_NAME}/ecto_cells
                         INSTALL
  ecto_tabletop.cpp
  wrap_tabletop_TableArray.cpp
)

link_ecto(ecto_tabletop
  ${OpenCV_LIBS}
  ${roscpp_LIBRARIES}
  ${rosbag_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  )

# create some messages for ROS

find_package(genmsg)
if (genmsg_FOUND)
  SET(TABLETOP_ROS_CELLS TableMsgAssembler.cpp TableVisualizationMsgAssembler.cpp)
endif()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

# deal with the detection TOD module
ectomodule(tabletop_table DESTINATION ${PROJECT_NAME}/ecto_cells
                          INSTALL
                          ClusterDrawer.cpp
                          Clusterer.cpp
  module.cpp
  TableDetector.cpp
  TablePose.cpp
  ${TABLETOP_ROS_CELLS}
)

link_ecto(tabletop_table ${Boost_LIBRARIES}
                         ${catkin_LIBRARIES}
                         ${OpenCV_LIBRARIES}
                         marker_generator
)

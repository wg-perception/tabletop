# create some messages for ROS
if (object_recognition_msgs_FOUND)
  SET(TABLETOP_ROS_CELLS TableMsgAssembler.cpp TableVisualizationMsgAssembler.cpp)
endif()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

# deal with the table detection module
ectomodule(tabletop_table DESTINATION ${PROJECT_NAME}/ecto_cells
                          INSTALL
  module.cpp
  TableDetector.cpp
  ${TABLETOP_ROS_CELLS}
)

link_ecto(tabletop_table ${Boost_LIBRARIES}
                         ${catkin_LIBRARIES}
                         ${OpenCV_LIBRARIES}
)

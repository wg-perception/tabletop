# create some messages for ROS
find_package(genmsg)
if (genmsg_FOUND)
#  find_package(ROS REQUIRED geometry_msgs sensor_msgs std_msgs)
#  add_message_files(DIRECTORY msg FILES Table.msg)
  #generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)
  #SET(TABLETOP_ROS_CELLS TablePublisher)
endif()

# deal with the detection TOD module
ectomodule(tabletop_table
  module.cpp
  TableFinder.cpp
  ${TABLETOP_ROS_CELLS}
)

link_ecto(tabletop_table
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

install_ecto_module(tabletop_table)

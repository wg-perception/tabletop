# deal with the detection TOD module
ectomodule(tabletop_table
  module.cpp
  TableFinder.cpp
  )

link_ecto(tabletop_table
  ${pcl_LIBRARIES}
  ${Boost_LIBRARIES}
)

#install_ecto_module(tabletop_table)

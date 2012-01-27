# create a small library for tabletop detection
add_library(tabletop_table
            tabletop_segmenter.cpp)

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

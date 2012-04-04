find_pcl_package()
find_package(Eigen)

include_directories(SYSTEM
                    ${PCL_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIRS}
)

add_library(pcl_latest
            organized_connected_component_segmentation.cpp
#            region_growing.cpp
# seeded_hue_segmentation.cpp
#            extract_clusters.cpp
            organized_multi_plane_segmentation.cpp
#region_growing_rgb.cpp
#segment_differences.cpp
#extract_polygonal_prism_data.cpp
#planar_polygon_fusion.cpp
#sac_segmentation.cpp
)

target_link_libraries(pcl_latest
                      ${PCL_LIBRARIES}
)


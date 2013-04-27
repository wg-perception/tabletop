add_library(marker_generator SHARED marker_generator.cpp)
add_dependencies(marker_generator tabletop_gencpp)
target_link_libraries(marker_generator ${catkin_LIBRARIES})

install(TARGETS marker_generator
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

include_directories(SYSTEM ${catkin_INLUDE_DIRS})

add_library(tabletop_model_fitter SHARED
    model_fitter.cpp
    iterative_distance_fitter.cpp
)

find_package(Boost REQUIRED)
target_link_libraries(tabletop_model_fitter
    marker_generator
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

install(TARGETS tabletop_model_fitter
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

ectomodule(tabletop_object DESTINATION ${PROJECT_NAME}/ecto_cells
                           INSTALL
    module.cpp
    ObjectRecognizer.cpp
)

link_ecto(tabletop_object ${catkin_LIBRARIES}
                          ${Boost_LIBRARIES}
                          ${PCL_LIBRARIES}
                          marker_generator
                          tabletop_model_fitter
                          tabletop_db
)

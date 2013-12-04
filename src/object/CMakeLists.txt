include_directories(SYSTEM ${catkin_INLUDE_DIRS})

add_library(tabletop_model_fitter SHARED
    model_fitter.cpp
    iterative_distance_fitter.cpp
)

find_package(Boost REQUIRED)
target_link_libraries(tabletop_model_fitter
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
                          tabletop_model_fitter
                          tabletop_db
)

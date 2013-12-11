FIND_PACKAGE(OpenGL)

find_package(PkgConfig REQUIRED)
find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()
if (NOT ASSIMP_FOUND)
  message(WARNING "ASsimp not found, not building synthetic views")
endif()
if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
  message(STATUS "Found assimp v3")
  set(EXTRA_SOURCES)
  set(EXTRA_LIBRARIES assimp)
else()
  message(STATUS "Building assimp v3")
  set(ASSIMP_INCLUDE_DIRS ./assimp/include)
  aux_source_directory(./assimp/contrib/clipper EXTRA_SOURCES_clipper)
  aux_source_directory(./assimp/contrib/ConvertUTF EXTRA_SOURCES_ConvertUTF)
  aux_source_directory(./assimp/contrib/irrXML EXTRA_SOURCES_irrXML)
  aux_source_directory(./assimp/contrib/poly2tri/poly2tri/common EXTRA_SOURCES_poly2tri)
  aux_source_directory(./assimp/contrib/poly2tri/poly2tri/sweep EXTRA_SOURCES_poly2tri_sweep)
  aux_source_directory(./assimp/contrib/unzip EXTRA_SOURCES_unzip)
  aux_source_directory(./assimp/contrib/zlib EXTRA_SOURCES_zlib)
  aux_source_directory(./assimp/code EXTRA_SOURCES)
  set(EXTRA_SOURCES ${EXTRA_SOURCES} ${EXTRA_SOURCES_clipper} ${EXTRA_SOURCES_ConvertUTF} ${EXTRA_SOURCES_irrXML} ${EXTRA_SOURCES_poly2tri} ${EXTRA_SOURCES_poly2tri_sweep} ${EXTRA_SOURCES_unzip} ${EXTRA_SOURCES_zlib})
  set(EXTRA_LIBRARIES)
endif()

include_directories(BEFORE ${ASSIMP_INCLUDE_DIRS})



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

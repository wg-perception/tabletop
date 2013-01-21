# pipeline tests
object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/config_detection.tabletop_table)
if (NOT ${household_objects_database_LIBRARIES} STREQUAL "household_objects_database_LIBRARIES-NOTFOUND")
object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/config_detection.tabletop_object)
endif()

# publisher tests
object_recognition_core_sink_test(TablePublisher "object_recognition_tabletop" "{}")

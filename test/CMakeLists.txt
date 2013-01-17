find_package(object_recognition_core)
if (object_recognition_core_FOUND)
    # pipeline tests
    object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/config_detection.tabletop_table)
    #object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/config_detection.tabletop_object)

    # publisher tests
    object_recognition_core_sink_test(TablePublisher "object_recognition_tabletop" "{}")
endif()

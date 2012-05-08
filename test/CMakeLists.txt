find_package(object_recognition_core)
if (object_recognition_core_FOUND)
    # pipeline tests
    object_recognition_core_detection_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/config_detection.tabletop_table)
    #object_recognition_core_detection_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/config_detection.tabletop_object)
    #object_recognition_training_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/config_training.transparent_objects)

    # publisher tests
    object_recognition_core_sink_test(table_publisher "{}" "object_recognition_tabletop")
endif()

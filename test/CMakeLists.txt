# pipeline tests
#object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/detection.table.ros.ork)
#object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/detection.object.ros.ork)
object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/detection.object.ork)
object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/detection.table.ork)

# publisher tests
object_recognition_core_sink_test(TablePublisher "object_recognition_tabletop" "{}")

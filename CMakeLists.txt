cmake_minimum_required(VERSION 2.8.3)
project(visy_detector_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs geometry_msgs sensor_msgs actionlib actionlib_msgs cv_bridge image_transport visy_neopixel_pkg)
find_package(OpenCV REQUIRED)

add_message_files(FILES MetalChip.msg ConveyorSystem.msg)

add_service_files(DIRECTORY srv FILES StartMetalChipDetector.srv StopMetalChipDetector.srv SelectImage.srv)

add_action_files(DIRECTORY action FILES DetectConveyor.action)

generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs actionlib_msgs cv_bridge image_transport visy_neopixel_pkg)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(conveyor_detector_node src/conveyor_detector_node.cpp)
target_link_libraries(conveyor_detector_node ${catkin_LIBRARIES})
target_link_libraries(conveyor_detector_node ${OpenCV_LIBS})
add_dependencies(conveyor_detector_node visy_detector_pkg_gencpp)
add_dependencies(conveyor_detector_node ${catkin_EXPORTED_TARGETS})
install(TARGETS conveyor_detector_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(metal_chip_detector_node src/metal_chip_detector_node.cpp)
target_link_libraries(metal_chip_detector_node ${catkin_LIBRARIES})
target_link_libraries(metal_chip_detector_node ${OpenCV_LIBS})
add_dependencies(metal_chip_detector_node visy_detector_pkg_gencpp)
add_dependencies(metal_chip_detector_node ${catkin_EXPORTED_TARGETS})
install(TARGETS metal_chip_detector_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


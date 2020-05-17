cmake_minimum_required(VERSION 2.8.3)
project(visy_detector_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge image_transport)
find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(conveyor_detector_node src/conveyor_detector_node.cpp)
target_link_libraries(conveyor_detector_node ${catkin_LIBRARIES})
target_link_libraries(conveyor_detector_node ${OpenCV_LIBS})
install(TARGETS conveyor_detector_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(metal_chip_detector_node src/metal_chip_detector_node.cpp)
target_link_libraries(metal_chip_detector_node ${catkin_LIBRARIES})
target_link_libraries(metal_chip_detector_node ${OpenCV_LIBS})
install(TARGETS metal_chip_detector_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


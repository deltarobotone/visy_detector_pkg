cmake_minimum_required(VERSION 2.8.3)
project(visy_detector_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp one_easy_protocol_pkg visy_neopixel_pkg sensor_msgs cv_bridge image_transport)
find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS one_easy_protocol_pkg visy_neopixel_pkg roscpp sensor_msgs cv_bridge image_transport)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(image_processing_node src/image_processing_node.cpp)
target_link_libraries(image_processing_node ${catkin_LIBRARIES})
target_link_libraries(image_processing_node ${OpenCV_LIBS})

install(TARGETS image_processing_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


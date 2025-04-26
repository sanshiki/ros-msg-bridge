cmake_minimum_required(VERSION 3.0.2)
project(ros-msg-bridge)

## complie options
add_compile_options(-std=c++17)

## rospackage
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
)

## opencv
find_package(OpenCV REQUIRED)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(img_convert_node src/img_convert_node.cpp)

target_link_libraries(img_convert_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

cmake_minimum_required(VERSION 3.8)
project(imx_stream)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ONNX Runtime setup
set(ONNXRUNTIME_DIR "/usr/local/onnxruntime")
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)

add_executable(camera_publisher src/camera_publisher.cpp)
ament_target_dependencies(camera_publisher rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(image_subscriber src/image_subscriber.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(yolo_inference src/yolo_inference.cpp)
target_link_libraries(yolo_inference ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so)
ament_target_dependencies(yolo_inference rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  camera_publisher
  image_subscriber
  yolo_inference
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(zbar_opencv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(ZBAR REQUIRED IMPORTED_TARGET zbar)

add_executable(zbar_opencv
  src/zbar_opencv.cpp
)

ament_target_dependencies(zbar_opencv
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

target_include_directories(zbar_opencv PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(zbar_opencv
  ${OpenCV_LIBRARIES}
  PkgConfig::ZBAR
)

install(TARGETS
  zbar_opencv
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

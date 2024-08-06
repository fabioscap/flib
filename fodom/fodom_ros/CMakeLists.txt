cmake_minimum_required(VERSION 3.8)
project(fodom_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(fodom REQUIRED)

add_executable(fodom_ros src/main.cpp)
target_link_libraries(fodom_ros fodom)
install(TARGETS fodom_ros DESTINATION lib/${PROJECT_NAME})
ament_package()
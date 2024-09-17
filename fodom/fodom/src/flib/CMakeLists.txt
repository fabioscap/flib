project(test_manifold)
cmake_minimum_required(VERSION 3.22)
add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(test manifold.cpp)
target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test PUBLIC Eigen3::Eigen)
cmake_minimum_required(VERSION 3.8)
project(fodom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package (Eigen3 3.3 REQUIRED NO_MODULE)


add_library(fodom SHARED 
            src/manifold.cpp
            src/model.cpp
            src/odom.cpp)
target_link_libraries(fodom PUBLIC Eigen3::Eigen)

target_include_directories(fodom
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the library and headers
install(TARGETS fodom
        EXPORT fodomTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(EXPORT fodomTargets
        FILE fodomConfig.cmake
        DESTINATION lib/cmake/fodom)

install(DIRECTORY include/ DESTINATION include)
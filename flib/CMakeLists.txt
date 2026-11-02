cmake_minimum_required(VERSION 3.22)
project(flib)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

file(GLOB HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/flib/*.h")
file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/flib/*.cpp")

add_library(flib ${SOURCE_FILES})
target_include_directories(flib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(flib PUBLIC Eigen3::Eigen)
install(FILES ${HEADER_FILES} DESTINATION include/flib)

install(EXPORT flibTargets
        FILE flibTargets.cmake
        DESTINATION lib/cmake/flib)
install(TARGETS flib
        EXPORT flibTargets
        DESTINATION lib
        INCLUDES DESTINATION include/flib)
install(FILES "flibConfig.cmake" DESTINATION lib/cmake/flib)

option(FLIB_BUILD_APPS "Build flib examples and benchmarks" ON)
if(FLIB_BUILD_APPS)
  add_subdirectory(app)
endif()

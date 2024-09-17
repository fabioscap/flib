cmake_minimum_required(VERSION 3.22)
project(flib)

add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

file(GLOB HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/flib/*.h")
file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/flib/*.cpp")

add_library(flib SHARED ${SOURCE_FILES})
target_include_directories(flib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(flib PUBLIC Eigen3::Eigen)

install(EXPORT flibTargets DESTINATION lib/cmake)
install(TARGETS flib
        EXPORT flibTargets
        DESTINATION lib
        INCLUDES DESTINATION include/flib)

install(FILES ${HEADER_FILES} DESTINATION include/flib)

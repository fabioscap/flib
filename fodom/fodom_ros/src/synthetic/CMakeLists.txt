cmake_minimum_required(VERSION 3.22)
project(synthetic)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (fodom REQUIRED)

file(GLOB HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/synthetic/*.h")
file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/synthetic/*.cpp")

add_library(synthetic SHARED ${SOURCE_FILES})
target_include_directories(synthetic PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(synthetic PUBLIC Eigen3::Eigen fodom)
install(FILES ${HEADER_FILES} DESTINATION include/synthetic)

install(EXPORT syntheticTargets
        FILE syntheticTargets.cmake
        DESTINATION lib/cmake/synthetic)
install(TARGETS synthetic
        EXPORT syntheticTargets
        DESTINATION lib
        INCLUDES DESTINATION include/synthetic)
install(FILES "syntheticConfig.cmake" DESTINATION lib/cmake/synthetic)


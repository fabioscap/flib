add_library(synthetic SHARED synthetic.cpp)
target_include_directories(synthetic
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(synthetic PUBLIC fodom Eigen3::Eigen)
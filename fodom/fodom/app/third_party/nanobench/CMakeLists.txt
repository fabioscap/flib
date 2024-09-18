project(nanobench)

add_library(nanobench SHARED nanobench.cpp)
target_include_directories(nanobench PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)

install(TARGETS nanobench
    EXPORT nanobenchTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(FILES nanobench.h DESTINATION include)

install(EXPORT nanobenchTargets
    FILE nanobenchTargets.cmake
    DESTINATION lib/cmake/nanobench)
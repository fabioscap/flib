add_subdirectory(third_party/nanobench)

add_executable(test_manifold test_manifold.cpp)
target_link_libraries(test_manifold PUBLIC flib nanobench)

add_executable(test_unscented test_unscented.cpp)
target_link_libraries(test_unscented PUBLIC flib)

add_executable(eigen_vs_manifold eigen_vs_manifold.cpp)
target_link_libraries(eigen_vs_manifold PUBLIC flib nanobench)

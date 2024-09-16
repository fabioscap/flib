add_executable(var var.cpp)
target_link_libraries(var PUBLIC fodom)

add_executable(eigen_vs_manifold eigen_vs_manifold.cpp)
target_link_libraries(eigen_vs_manifold PUBLIC fodom)

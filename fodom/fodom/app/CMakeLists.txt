# add_executable(var var.cpp)
# target_link_libraries(var PUBLIC fodom)


add_subdirectory(third_party/nanobench)
add_subdirectory(synthetic)

add_executable(test_manifold test_manifold.cpp)
target_link_libraries(test_manifold PUBLIC flib nanobench synthetic)

add_executable(test_unscented test_unscented.cpp)
target_link_libraries(test_unscented PUBLIC flib nanobench)
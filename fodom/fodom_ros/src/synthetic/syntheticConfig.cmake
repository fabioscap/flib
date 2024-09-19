include(CMakeFindDependencyMacro)
find_dependency(Eigen3 3.3 REQUIRED NO_MODULE)
find_dependency(fodom REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/syntheticTargets.cmake")
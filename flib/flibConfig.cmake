include(CMakeFindDependencyMacro)
find_dependency(Eigen3 3.3 REQUIRED NO_MODULE)

include("${CMAKE_CURRENT_LIST_DIR}/flibTargets.cmake")
add_library(icp STATIC icp.cpp kdtree.cpp)
target_include_directories(icp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_features(icp PUBLIC cxx_std_20)
target_link_libraries(icp PUBLIC Eigen3::Eigen)
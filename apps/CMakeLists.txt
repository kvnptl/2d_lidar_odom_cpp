add_executable(main main.cpp)
target_link_libraries(main PRIVATE dataloader viewer)
target_compile_features(main PRIVATE cxx_std_17)

add_executable(testing testing.cpp)
target_link_libraries(testing PRIVATE dataloader viewer)
target_compile_features(testing PRIVATE cxx_std_17)

add_executable(bnn basic_nearest_neighbor.cpp)
target_link_libraries(bnn PRIVATE dataloader viewer)
target_compile_features(bnn PRIVATE cxx_std_17)

# Add the nanoflann include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../nanoflann/include)

add_executable(kdtree_nanoflann kd_tree_based_nanoflann.cpp)
target_link_libraries(kdtree_nanoflann PRIVATE dataloader viewer)
target_compile_features(kdtree_nanoflann PRIVATE cxx_std_17)

add_executable(kdtree kd_tree_scratch.cpp)
target_link_libraries(kdtree PRIVATE dataloader viewer)
target_compile_features(kdtree PRIVATE cxx_std_17)

add_executable(main_kd_tree_nanoflann main_kd_tree_nanoflann.cpp)
target_link_libraries(main_kd_tree_nanoflann PRIVATE dataloader viewer)
target_compile_features(main_kd_tree_nanoflann PRIVATE cxx_std_17)

add_executable(main_pt1_kd_tree main_pt1_kd_tree.cpp)
target_link_libraries(main_pt1_kd_tree PRIVATE dataloader viewer)
target_compile_features(main_pt1_kd_tree PRIVATE cxx_std_17)

add_executable(main_pt2_error main_pt2_error.cpp)
target_link_libraries(main_pt2_error PRIVATE dataloader viewer)
target_compile_features(main_pt2_error PRIVATE cxx_std_17)

add_executable(main_pt3_jacobian main_pt3_jacobian.cpp)
target_link_libraries(main_pt3_jacobian PRIVATE dataloader viewer)
target_compile_features(main_pt3_jacobian PRIVATE cxx_std_17)
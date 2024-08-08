add_executable(main main.cpp)
target_link_libraries(main PRIVATE dataloader viewer)
target_compile_features(main PRIVATE cxx_std_20)

# Add the nanoflann include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../nanoflann/include)

# add_executable(testing testing.cpp)
# target_link_libraries(testing PRIVATE dataloader viewer)
# target_compile_features(testing PRIVATE cxx_std_20)

# add_executable(basic_nearest_neighbor basic_nearest_neighbor.cpp)
# target_link_libraries(basic_nearest_neighbor PRIVATE dataloader viewer)
# target_compile_features(basic_nearest_neighbor PRIVATE cxx_std_20)

# add_executable(kd_tree_based_nanoflann kd_tree_based_nanoflann.cpp)
# target_link_libraries(kd_tree_based_nanoflann PRIVATE dataloader viewer)
# target_compile_features(kd_tree_based_nanoflann PRIVATE cxx_std_20)

# add_executable(kd_tree_scratch kd_tree_scratch.cpp)
# target_link_libraries(kd_tree_scratch PRIVATE dataloader viewer)
# target_compile_features(kd_tree_scratch PRIVATE cxx_std_20)

# add_executable(main_kd_tree_nanoflann main_kd_tree_nanoflann.cpp)
# target_link_libraries(main_kd_tree_nanoflann PRIVATE dataloader viewer)
# target_compile_features(main_kd_tree_nanoflann PRIVATE cxx_std_20)

# add_executable(main_pt1_kd_tree main_pt1_kd_tree.cpp)
# target_link_libraries(main_pt1_kd_tree PRIVATE dataloader viewer)
# target_compile_features(main_pt1_kd_tree PRIVATE cxx_std_20)

# add_executable(main_pt2_error main_pt2_error.cpp)
# target_link_libraries(main_pt2_error PRIVATE dataloader viewer)
# target_compile_features(main_pt2_error PRIVATE cxx_std_20)

# add_executable(main_pt3_jacobian main_pt3_jacobian.cpp)
# target_link_libraries(main_pt3_jacobian PRIVATE dataloader viewer)
# target_compile_features(main_pt3_jacobian PRIVATE cxx_std_20)

# add_executable(main_pt4_hessian main_pt4_hessian.cpp)
# target_link_libraries(main_pt4_hessian PRIVATE dataloader viewer)
# target_compile_features(main_pt4_hessian PRIVATE cxx_std_20)

# add_executable(main_pt5_final main_pt5_final.cpp)
# target_link_libraries(main_pt5_final PRIVATE dataloader viewer)
# target_compile_features(main_pt5_final PRIVATE cxx_std_20)

# add_executable(main_pt5v1_final main_pt5v1_final.cpp)
# target_link_libraries(main_pt5v1_final PRIVATE dataloader viewer)
# target_compile_features(main_pt5v1_final PRIVATE cxx_std_20)

add_executable(main_pt6_2d_lidar_odom main_pt6_2d_lidar_odom.cpp)
target_link_libraries(main_pt6_2d_lidar_odom PRIVATE dataloader viewer)
target_compile_features(main_pt6_2d_lidar_odom PRIVATE cxx_std_20)

add_executable(main_pt6_2d_lidar_odom_nanoflann main_pt6_2d_lidar_odom_nanoflann.cpp)
target_link_libraries(main_pt6_2d_lidar_odom_nanoflann PRIVATE dataloader viewer)
target_compile_features(main_pt6_2d_lidar_odom_nanoflann PRIVATE cxx_std_20)

add_executable(main_pt6v2_2d_lidar_odom main_pt6v2_2d_lidar_odom.cpp)
target_link_libraries(main_pt6v2_2d_lidar_odom PRIVATE dataloader viewer icp)
target_compile_features(main_pt6v2_2d_lidar_odom PRIVATE cxx_std_20)

add_executable(main_visualize main_visualize.cpp)
target_link_libraries(main_visualize PRIVATE dataloader viewer icp)
target_compile_features(main_visualize PRIVATE cxx_std_20)

add_executable(main_pt7_2d_lidar_odom main_pt7_2d_lidar_odom.cpp)
target_link_libraries(main_pt7_2d_lidar_odom PRIVATE dataloader viewer icp)
target_compile_features(main_pt7_2d_lidar_odom PRIVATE cxx_std_20)

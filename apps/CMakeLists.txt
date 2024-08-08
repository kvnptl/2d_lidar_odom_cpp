add_executable(main main.cpp)
target_link_libraries(main PRIVATE dataloader viewer icp)
target_compile_features(main PRIVATE cxx_std_20)
add_executable(main main.cpp)
target_link_libraries(main PRIVATE dataloader viewer icp)
target_compile_features(main PRIVATE cxx_std_20)

add_executable(main_modular main_modular.cpp)
target_link_libraries(main_modular PRIVATE dataloader viewer icp)
target_compile_features(main_modular PRIVATE cxx_std_20)
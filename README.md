### Modern C++ 2024 Final Project: 2D Lidar odometry in the Real-World

This project implements 2D LiDAR odometry using the ICP (Iterative Closest Point) algorithm. It processes 2D LiDAR scans collected from a moving robot in a corridor. The project employs a KD-Tree based data structure for finding correspondences and offers a modular structure for the ICP algorithm, dataloader, and visualizer.

<!-- TODO: Add if possible GIF of the ICP algorithm-->

| ![Figure 1: f1](media/before_icp.png) | ![Figure 2: f2](media/after_icp.png) |
|:--:|:--:|
| *Figure 1: Before ICP* | *Figure 2: After ICP* |

## Required Libraries

- Eigen
  - Install with `sudo apt install libeigen3-dev`
- Open3D
  - Visit [Open3d github](https://github.com/isl-org/Open3D/releases)
    - Download file `open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz` from the Assets section
  - Extract the downloaded ‘tar’ file to the project folder, rename the folder as simply `open3d`

## Project Structure

> NOTE: Keep the all the binary files data in the `BINARY` folder

```plaintext
|-- 2d_lidar_odom_cpp
|   |-- apps
|   |   |-- main.cpp
|   |   |-- CMakeLists.txt
|   |-- BINARY
|   |-- dataloader
|   |   |-- CMakeLists.txt
|   |   |-- dataloader.cpp
|   |   |-- dataloader.h
|   |-- icp
|   |   |-- CMakeLists.txt
|   |   |-- icp.cpp
|   |   |-- icp.hpp
|   |   |-- kdtree.cpp
|   |   |-- kdtree.hpp
|   |-- open3d
|   |   |-- include
|   |   |-- lib
|   |   |-- share
|   |-- viewer
|   |   |-- CMakeLists.txt
|   |   |-- viewer.cpp
|   |   |-- viewer.hpp
|   |-- CMakeLists.txt
|   |-- README.md
|   |-- .gitignore
```

## How to run the project

1. Build the project (from the project root directory)

- `cmake -Bbuild .` - Generates build system files in the build directory
- `cmake --build build` - Compiles the project using the build system files in the build directory

2. Run the project

- `./build/apps/main .`

### Troubleshooting Build Errors

If you receive the error `/usr/bin/ld: warning: libc++.so.1, needed by ../../open3d/lib/libOpen3D.so, not found` while building, follow these steps:

- Check the output of the terminal command `$ find /usr/lib -name "libc++.so.1"`. This command should print the path to libc++.
- If libc++ is not found, run the script `install_deps.sh` (NOTE: ensure the script is executable)

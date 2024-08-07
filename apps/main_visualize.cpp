#include "dataloader/dataloader.hpp"
#include "viewer/viewer.hpp"
#include <chrono>
#include <iostream>
#include <open3d/Open3D.h>
#include <thread>

int main(int argc, char const *argv[])
{
    if (argc != 2)
    {
        std::cerr << "[ERROR] Please provide the path to the dataset directory" << std::endl;
        return 1;
    }
    const std::string &filename(argv[1]);

    dataset::LaserScanDataset pointcloud_set(filename);

    // Get the size of the pointcloud set
    size_t size = pointcloud_set.size();
    std::cout << "Size of pointcloud set: " << size << std::endl;

    // Create a visualizer
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Point Cloud Viewer", 800, 600);

    for (size_t i = 0; i < size - 1; ++i)
    {
        auto first_scan = pointcloud_set[i];
        auto second_scan = pointcloud_set[i + 1];

        viewer::viewTwoClouds(first_scan, second_scan, visualizer);

        // Add a delay between visualizations (e.g., 1 second)
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(33)));
    }

    // Destroy the visualizer window after the loop
    visualizer.DestroyVisualizerWindow();

    return 0;
}
#include "dataloader/dataloader.hpp"
#include "icp/icp.hpp"
#include "viewer/viewer.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <vector>

int main(int argc, char const *argv[])
{
    if (argc != 2)
    {
        std::cerr << "[ERROR] Please provide the path to the dataset directory" << std::endl;
        return 1;
    }
    const std::string &filename(argv[1]);

    dataset::LaserScanDataset pointcloud_set(filename);
    std::cout << "[INFO] Size of pointcloud set: " << pointcloud_set.size() << std::endl;

    Eigen::Matrix2d R_total = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t_total = Eigen::Vector2d::Zero();

    double running_average_time = 0.0;
    int num_processed_scans = 0;

    // Create a visualizer
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Point Cloud Viewer", 800, 600);

    double voxel_size = 0.1;
    std::vector<Eigen::Vector2d> source_scan = voxelGridDownsample(pointcloud_set[0], voxel_size);

    for (size_t scan_idx = 1; scan_idx < pointcloud_set.size() - 1 - 6000; ++scan_idx)
    {
        std::vector<Eigen::Vector2d> target_scan = voxelGridDownsample(pointcloud_set[scan_idx], voxel_size);

        std::cout << "[INFO] Processing scans " << scan_idx << " and " << scan_idx + 1 << std::endl;

        // Start with identity rotation and zero translation
        Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
        Eigen::Vector2d t = Eigen::Vector2d::Zero();

        auto start_time = std::chrono::high_resolution_clock::now();

        runICP(source_scan, target_scan, R, t);

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;

        ++num_processed_scans;
        running_average_time += (duration.count() - running_average_time) / num_processed_scans;

        std::cout << "[INFO] Running average ICP time after " << num_processed_scans << " scan pairs: " << 1.0 / running_average_time << " Hz" << std::endl;

        // Accumulate the total transformation
        R_total = R * R_total;
        t_total = R * t_total + t;

        // Apply final transformation to the current source scan points and overwrite source_scan
        for (auto &point : source_scan)
        {
            point = R * point + t;
        }

        // Concatenate the transformed source scan with the target scan
        source_scan.insert(source_scan.end(), target_scan.begin(), target_scan.end());

        // Downsample the source scan
        source_scan = voxelGridDownsample(source_scan, voxel_size);

        viewer::viewCloud(source_scan, visualizer);

        target_scan.clear();
    }

    // Destroy the visualizer window after the loop
    visualizer.DestroyVisualizerWindow();

    // Output the final accumulated transformation
    std::cout << "[INFO] Final accumulated transformation:" << std::endl;
    std::cout << "R_total = \n"
              << R_total << std::endl;
    std::cout << "t_total = \n"
              << t_total.transpose() << std::endl;

    return 0;
}

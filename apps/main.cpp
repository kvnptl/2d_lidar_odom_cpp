#include "dataloader/dataloader.hpp"
#include "icp/icp.hpp"
#include "viewer/viewer.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

using std::cerr;
using std::cout;
using std::endl;

int main(int argc, char const *argv[])
{
    if (argc != 2)
    {
        cerr << "[ERROR] Please provide the path to the dataset directory" << endl;
        return 1;
    }
    const std::string &filename(argv[1]);

    dataset::LaserScanDataset pointcloud_set(filename);
    const size_t total_scans = pointcloud_set.size() - 1;
    cout << "[INFO] Size of pointcloud set: " << total_scans << endl;

    Eigen::Matrix2d R_total = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t_total = Eigen::Vector2d::Zero();

    double running_average_time = 0.0;
    int num_processed_scans = 0;

    // Create a visualizer
    // open3d::visualization::Visualizer visualizer;
    // visualizer.CreateVisualizerWindow("Point Cloud Viewer", 800, 600);

    const double voxel_size = 0.2;
    std::vector<Eigen::Vector2d> source_scan = icp::downsample(pointcloud_set[0], voxel_size);

    for (size_t scan_idx = 1; scan_idx < total_scans; scan_idx++)
    {
        std::vector<Eigen::Vector2d> target_scan = icp::downsample(pointcloud_set[scan_idx], voxel_size);

        // cout << "[INFO] Processing scans " << scan_idx << " and " << scan_idx + 1 << endl;

        // Start with identity rotation and zero translation
        Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
        Eigen::Vector2d t = Eigen::Vector2d::Zero();

        auto start_time = std::chrono::high_resolution_clock::now();

        icp::runICP(source_scan, target_scan, R, t);

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;

        ++num_processed_scans;
        running_average_time += (duration.count() - running_average_time) / num_processed_scans;

        int percentage_done = static_cast<int>((static_cast<double>(scan_idx) / total_scans) * 100);

        if (scan_idx % 10 == 0)
        {
            cout << "[INFO] Avg ICP time: " << std::fixed << std::setprecision(2) << 1.0 / running_average_time << " Hz | " << "Progress: " << percentage_done << "%" << endl;
        }

        // Accumulate the total transformation
        R_total = R * R_total;
        t_total = R * t_total + t;

        // Transform the source scan
        std::transform(source_scan.begin(), source_scan.end(), source_scan.begin(),
                       [&R, &t](auto &point)
                       {
                           return R * point + t;
                       });

        // Concatenate the transformed source scan with the target scan
        source_scan.insert(source_scan.end(), target_scan.begin(), target_scan.end());

        // Downsample the source scan
        source_scan = icp::downsample(source_scan, voxel_size);

        // viewer::viewCloud(source_scan, visualizer);

        target_scan.clear();
    }

    // Destroy the visualizer window after the loop
    // visualizer.DestroyVisualizerWindow();

    // Output the final accumulated transformation
    cout << "[INFO] Final accumulated transformation:" << endl;
    cout << "R_total = \n"
         << R_total << endl;
    cout << "t_total = \n"
         << t_total.transpose() << endl;

    return 0;
}

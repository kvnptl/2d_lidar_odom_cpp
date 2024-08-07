#include "dataloader/dataloader.hpp"
#include "icp/icp.hpp"
#include "viewer/viewer.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

using Eigen::Matrix2d;
using Eigen::Vector2d;

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

    Matrix2d R_total = Matrix2d::Identity();
    Vector2d t_total = Vector2d::Zero();

    double running_average_time = 0.0;
    int num_processed_scans = 0;

    for (size_t scan_idx = 0; scan_idx < pointcloud_set.size() - 1; ++scan_idx)
    {
        std::vector<Vector2d> target_scan = pointcloud_set[scan_idx];
        std::vector<Vector2d> source_scan = pointcloud_set[scan_idx + 1];

        std::cout << "[INFO] Processing scans " << scan_idx << " and " << scan_idx + 1 << std::endl;

        computeICP(target_scan, source_scan, R_total, t_total, running_average_time, num_processed_scans);

        std::vector<Vector2d> transformed_source_scan;
        for (const auto &point : source_scan)
        {
            transformed_source_scan.push_back(R_total * point + t_total);
        }
    }

    std::cout << "[INFO] Final accumulated transformation:" << std::endl;
    std::cout << "R_total = \n"
              << R_total << std::endl;
    std::cout << "t_total = \n"
              << t_total.transpose() << std::endl;

    return 0;
}

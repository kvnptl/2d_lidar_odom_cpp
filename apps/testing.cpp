#include "dataloader/dataloader.hpp"
#include "viewer/viewer.hpp"
#include <iostream>
#include <vector>

int main(int argc, char const *argv[])
{
    // if (argc != 2)
    // {
    //     std::cerr << "[ERROR] Please provide the path to the root directory of the dataset" << std::endl;
    //     return 1;
    // }
    // const std::string &filename(argv[1]);

    // dataset::LaserScanDataset pointcloud_set(filename);

    // auto first_scan = pointcloud_set[4050];

    // viewer::viewCloud(first_scan);

    // // Get the size of the pointcloud set
    // std::cout << "Size of pointcloud set: " << pointcloud_set.size() << std::endl;

    // // Get the number of points in the first scan
    // std::cout << "Number of points in the first scan: " << first_scan.size() << std::endl;

    std::vector<int> a = {1, 2, 3, 4, 5};

    std::cout << "Size of  a: " << a.size() << std::endl;

    return 0;
}
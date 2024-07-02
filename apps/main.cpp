#include "dataloader/dataloader.hpp"
#include "viewer/viewer.hpp"
#include <iostream>

int main(int argc, char const *argv[])
{
    if (argc != 2)
    {
        std::cerr << "[ERROR] Please provide the path to the root directory of the dataset" << std::endl;
        return 1;
    }
    const std::string &filename(argv[1]);

    dataset::LaserScanDataset pointcloud_set(filename);

    viewer::viewCloud(pointcloud_set[0]);
    return 0;
}
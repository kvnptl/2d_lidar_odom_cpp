#include "dataloader/dataloader.hpp"
#include "viewer/viewer.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <nanoflann.hpp>
#include <vector>

using Eigen::Vector2d;

// Define a structure to use with nanoflann for KD-Tree creation
struct PointCloud
{
    const std::vector<Vector2d> &pts;

    PointCloud(const std::vector<Vector2d> &pts) : pts(pts) {}

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" squared.
    inline double kdtree_distance(const double *p1, const size_t idx_p2, size_t /*size*/) const
    {
        const double d0 = p1[0] - pts[idx_p2][0];
        const double d1 = p1[1] - pts[idx_p2][1];
        return d0 * d0 + d1 * d1;
    }

    // Returns the dim'th component of the idx'th point in the class
    inline double kdtree_get_pt(const size_t idx, int dim) const
    {
        return pts[idx][dim];
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
};

int main(int argc, char const *argv[])
{
    if (argc != 2)
    {
        std::cerr << "[ERROR] Please provide the path to the dataset directory" << std::endl;
        return 1;
    }
    const std::string &filename(argv[1]);

    dataset::LaserScanDataset pointcloud_set(filename);

    std::vector<Vector2d> target_scan = pointcloud_set[0];
    std::vector<Vector2d> source_scan = pointcloud_set[1];

    // Get the size of the pointcloud set
    std::cout << "Size of pointcloud set: " << pointcloud_set.size() << std::endl;

    // Get the number of points in the source scan
    std::cout << "Number of points in the source scan: " << source_scan.size() << std::endl;

    // Construct a kd-tree index:
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloud>,
        PointCloud,
        2 /* dim */>
        my_kd_tree_t;

    PointCloud cloud(target_scan);
    my_kd_tree_t index(2 /*dim*/, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    index.buildIndex();

    std::vector<size_t> ret_index(1);
    std::vector<double> out_dist_sqr(1);
    nanoflann::KNNResultSet<double> resultSet(1);

    std::vector<std::pair<size_t, size_t>> correspondences;

    for (size_t i = 0; i < source_scan.size(); ++i)
    {
        resultSet.init(&ret_index[0], &out_dist_sqr[0]);
        index.findNeighbors(resultSet, &source_scan[i][0], nanoflann::SearchParameters(10));

        // Store the correspondence (index from source, index from target)
        correspondences.push_back(std::make_pair(i, ret_index[0]));
    }

    std::cout << "Found " << correspondences.size() << " correspondences." << std::endl;
    // for (const auto &corr : correspondences)
    // {
    //     std::cout << "Source index: " << corr.first << " corresponds to Target index: " << corr.second << std::endl;
    // }

    return 0;
}

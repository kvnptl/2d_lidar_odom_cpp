#include <Eigen/Core>
#include <iostream>
#include <nanoflann.hpp>
#include <vector>

// Define a type alias for convenience
using PointCloud = std::vector<Eigen::Vector2d>;

// KD-Tree adaptor for Eigen::Vector2d
struct PointCloudAdaptor
{
    const PointCloud &points;

    PointCloudAdaptor(const PointCloud &points) : points(points) {}

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return points.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class
    inline double kdtree_distance(const double *p1, const size_t idx_p2, size_t /*size*/) const
    {
        const Eigen::Vector2d &p2 = points[idx_p2];
        return (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]);
    }

    // Returns the dim'th component of the idx'th point in the class
    inline double kdtree_get_pt(const size_t idx, int dim) const
    {
        return points[idx][dim];
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
};

// Function to find the nearest neighbor correspondences using KD-Tree
std::vector<std::pair<int, int>> findCorrespondences(const PointCloud &scan1, const PointCloud &scan2)
{
    std::vector<std::pair<int, int>> correspondences;

    // Build KD-Tree for scan2
    PointCloudAdaptor adaptor(scan2);
    nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor>,
        PointCloudAdaptor,
        2 /* dim */
        >
        kdtree(2, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    kdtree.buildIndex();

    for (int i = 0; i < scan1.size(); ++i)
    {
        const double query_pt[2] = {scan1[i][0], scan1[i][1]};
        size_t ret_index;
        double out_dist_sqr;
        nanoflann::KNNResultSet<double> resultSet(1);
        resultSet.init(&ret_index, &out_dist_sqr);
        kdtree.findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));

        correspondences.emplace_back(i, ret_index);
    }

    return correspondences;
}

int main()
{
    // Example point clouds (replace with actual data)
    PointCloud scan1 = {
        {0.0, 0.0},
        {1.0, 1.0},
        {2.0, 2.0}};

    PointCloud scan2 = {
        {2.1, 2.1},
        {0.1, 0.1},
        {1.1, 1.1},
    };

    // Find correspondences
    auto correspondences = findCorrespondences(scan1, scan2);

    // Print correspondences
    for (const auto &correspondence : correspondences)
    {
        std::cout << "Scan1 point " << correspondence.first << " corresponds to Scan2 point " << correspondence.second << std::endl;
    }

    return 0;
}
#include "dataloader/dataloader.hpp"
#include "viewer/viewer.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using Eigen::Vector2d;

class KDNode
{
public:
    Vector2d point;
    KDNode *left;
    KDNode *right;

    KDNode(Vector2d pt) : point(pt), left(nullptr), right(nullptr) {}
};

class KDTree
{
private:
    KDNode *root;

    KDNode *build(std::vector<Vector2d>::iterator start, std::vector<Vector2d>::iterator end, int depth)
    {
        if (start >= end)
            return nullptr;

        int axis = depth % 2;
        std::sort(start, end, [axis](const Vector2d &a, const Vector2d &b)
                  { return a[axis] < b[axis]; });

        auto mid = start + (end - start) / 2;
        KDNode *node = new KDNode(*mid);
        node->left = build(start, mid, depth + 1);
        node->right = build(mid + 1, end, depth + 1);

        return node;
    }

    void nearest(KDNode *node, const Vector2d &target, KDNode *&best, double &best_dist, int depth)
    {
        if (!node)
            return;

        double d = (node->point - target).squaredNorm();
        if (d < best_dist)
        {
            best_dist = d;
            best = node;
        }

        int axis = depth % 2;
        KDNode *near = target[axis] < node->point[axis] ? node->left : node->right;
        KDNode *far = target[axis] < node->point[axis] ? node->right : node->left;

        nearest(near, target, best, best_dist, depth + 1);

        double dx = target[axis] - node->point[axis];
        if (dx * dx < best_dist)
        {
            nearest(far, target, best, best_dist, depth + 1);
        }
    }

public:
    KDTree(const std::vector<Vector2d> &points)
    {
        std::vector<Vector2d> mutable_points = points; // Make a copy of the points
        root = build(mutable_points.begin(), mutable_points.end(), 0);
    }

    Vector2d find_nearest(const Vector2d &target)
    {
        KDNode *best = nullptr;
        double best_dist = std::numeric_limits<double>::max();
        nearest(root, target, best, best_dist, 0);
        return best ? best->point : Vector2d();
    }
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

    KDTree tree(target_scan);

    std::vector<std::pair<size_t, Vector2d>> correspondences;

    for (size_t i = 0; i < source_scan.size(); ++i)
    {
        Vector2d nearest = tree.find_nearest(source_scan[i]);
        correspondences.push_back(std::make_pair(i, nearest));
    }

    std::cout << "Found " << correspondences.size() << " correspondences." << std::endl;
    // for (const auto &corr : correspondences)
    // {
    //     std::cout << "Source index: " << corr.first << " corresponds to Target point: " << corr.second.transpose() << std::endl;
    // }

    return 0;
}

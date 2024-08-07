#include "dataloader/dataloader.hpp"
#include "viewer/viewer.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;

// Class representing a node in the KD-Tree.
// Each node contains a point in 2D space, and links to left and right child nodes.
class KDNode
{
public:
    Vector2d point; // The 2D point stored in this node.
    KDNode *left;   // Pointer to the left child node.
    KDNode *right;  // Pointer to the right child node.

    // Constructor that initializes the node with a point.
    KDNode(const Vector2d &pt) : point(pt), left(nullptr), right(nullptr) {}
};

// Class representing the KD-Tree.
// The tree is used to efficiently find the nearest neighbor of a given point in 2D space.
class KDTree
{
private:
    KDNode *root; // Root node of the KD-Tree.

    // Recursive function to build the KD-Tree from a list of points.
    // 'start' and 'end' are iterators defining the current subset of points.
    // 'depth' indicates the current depth in the tree, used to choose the axis for splitting.
    KDNode *build(std::vector<Vector2d>::iterator start, std::vector<Vector2d>::iterator end, int depth)
    {
        if (start >= end)
            return nullptr; // Base case: no points to process.

        int axis = depth % 2; // Alternate between x (0) and y (1) axis for splitting.
        std::sort(start, end, [axis](const Vector2d &a, const Vector2d &b)
                  { return a[axis] < b[axis]; });

        auto mid = start + (end - start) / 2;         // Find the median element.
        KDNode *node = new KDNode(*mid);              // Create a node for the median element.
        node->left = build(start, mid, depth + 1);    // Recursively build the left subtree.
        node->right = build(mid + 1, end, depth + 1); // Recursively build the right subtree.

        return node;
    }

    // Recursive function to find the nearest neighbor of a given target point.
    // 'node' is the current node in the tree.
    // 'target' is the point we are finding the nearest neighbor for.
    // 'best' and 'bestDist' store the current best nearest neighbor and its distance squared.
    // 'depth' indicates the current depth in the tree.
    void nearest(KDNode *node, const Vector2d &target, KDNode *&best, double &bestDist, int depth)
    {
        if (!node)
            return; // Base case: reached a leaf node.

        double dist = (node->point - target).squaredNorm(); // Compute squared distance.
        if (dist < bestDist)
        { // Update the best nearest neighbor if closer.
            bestDist = dist;
            best = node;
        }

        int axis = depth % 2; // Determine the current axis.
        KDNode *near = target[axis] < node->point[axis] ? node->left : node->right;
        KDNode *far = target[axis] < node->point[axis] ? node->right : node->left;

        nearest(near, target, best, bestDist, depth + 1); // Search in the nearer subtree first.

        double dx = target[axis] - node->point[axis]; // Distance along the splitting axis.
        if (dx * dx < bestDist)
        { // Check if the farther subtree could contain a closer point.
            nearest(far, target, best, bestDist, depth + 1);
        }
    }

public:
    // Constructor that takes a vector of points and builds the KD-Tree.
    KDTree(const std::vector<Vector2d> &points)
    {
        std::vector<Vector2d> mutable_points = points; // Make a copy of the points
        root = build(mutable_points.begin(), mutable_points.end(), 0);
    }

    // Function to find the nearest neighbor of a given target point in the KD-Tree.
    Vector2d findNearest(const Vector2d &target)
    {
        KDNode *best = nullptr;
        double bestDist = std::numeric_limits<double>::max(); // Initialize best distance to maximum possible.
        nearest(root, target, best, bestDist, 0);             // Start the nearest neighbor search.
        return best ? best->point : Vector2d();               // Return the nearest point found, or zero if none found.
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
    std::cout << "[INFO] Size of pointcloud set: " << pointcloud_set.size() << std::endl;

    // Get the number of points in the source scan
    std::cout << "[INFO] Number of points in the source scan: " << source_scan.size() << std::endl;

    KDTree tree(target_scan);

    std::vector<std::pair<Vector2d, Vector2d>> correspondences; // Store pairs of source and nearest target points.
    std::vector<double> errors;                                 // To store the error for each correspondence.
    std::vector<MatrixXd> jacobians;                            // To store the Jacobian for each correspondence.
    // Assuming initial transformation is identity (R = I, t = 0)
    MatrixXd R = MatrixXd::Identity(2, 2);
    Vector2d t = Vector2d::Zero();

    for (const auto &source_point : source_scan)
    {
        Vector2d nearest_target = tree.findNearest(source_point);
        correspondences.push_back(std::make_pair(source_point, nearest_target));
        double error = (source_point - nearest_target).norm();
        errors.push_back(error);

        // Jacobian calculation:
        // Compute Rp_i + t
        Vector2d transformed_source = R * source_point + t;
        double x_hat = transformed_source[0];
        double y_hat = transformed_source[1];

        MatrixXd J(2, 3);
        J(0, 0) = 1;
        J(0, 1) = 0;
        J(0, 2) = -y_hat; // Partial derivatives with respect to x, y, theta
        J(1, 0) = 0;
        J(1, 1) = 1;
        J(1, 2) = x_hat; // Partial derivatives with respect to x, y, theta

        jacobians.push_back(J);
    }

    // Output results
    std::cout << "[INFO] Found " << correspondences.size() << " correspondences with the following errors and Jacobian matrices:" << std::endl;
    for (size_t i = 0; i < errors.size() && i < 5; ++i)
    {
        std::cout << "[INFO] Correspondence " << i + 1 << ": Error = " << errors[i] << std::endl;
        std::cout << "[INFO] Jacobian: " << jacobians[i] << std::endl;
        std::cout << std::endl;
    }

    return 0;
}

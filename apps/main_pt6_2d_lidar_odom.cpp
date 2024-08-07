#include "dataloader/dataloader.hpp"
#include "viewer/viewer.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using std::cout;
using std::endl;

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
    ~KDNode()
    {
        delete left;
        delete right;
    }
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

    ~KDTree()
    {
        delete root;
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
        std::cerr << "[ERROR] Please provide the path to the dataset directory" << endl;
        return 1;
    }
    const std::string &filename(argv[1]);

    dataset::LaserScanDataset pointcloud_set(filename);

    // viewer::viewTwoClouds(target_scan, source_scan);

    cout << "[INFO] Size of pointcloud set: " << pointcloud_set.size() << endl;

    Matrix2d R_total = Matrix2d::Identity();
    Vector2d t_total = Vector2d::Zero();

    double running_average_time = 0.0;
    int num_processed_scans = 0;

    for (size_t scan_idx = 0; scan_idx < pointcloud_set.size() - 1; ++scan_idx)
    {
        std::vector<Vector2d> target_scan = pointcloud_set[scan_idx];
        std::vector<Vector2d> source_scan = pointcloud_set[scan_idx + 1];

        cout << "[INFO] Processing scans " << scan_idx << " and " << scan_idx + 1 << endl;

        KDTree tree(target_scan);

        // Start with identity rotation and zero translation
        Matrix2d R = Matrix2d::Identity();
        Vector2d t = Vector2d::Zero();

        const int max_iterations = 100;
        const double tolerance = 1e-3;
        double prev_error = std::numeric_limits<double>::max();

        auto start_time = std::chrono::high_resolution_clock::now();

        for (int iter = 0; iter < max_iterations; ++iter)
        {
            std::vector<std::pair<Vector2d, Vector2d>> correspondences; // Store pairs of source and nearest target points.
            std::vector<double> errors;                                 // To store the error for each correspondence.
            std::vector<Matrix<double, 2, 3>> jacobians;                // To store the Jacobian for each correspondence.
            Matrix3d H = Matrix3d::Zero();
            Vector3d b = Vector3d::Zero();

            for (const auto &source_point : source_scan)
            {
                Vector2d transformed_point = R * source_point + t;
                Vector2d nearest_target = tree.findNearest(transformed_point);

                double error = (transformed_point - nearest_target).norm();
                errors.push_back(error);

                Vector2d transformed_source = R * source_point + t;
                double x_hat = transformed_source[0];
                double y_hat = transformed_source[1];

                Matrix<double, 2, 3> J;
                J(0, 0) = 1;
                J(0, 1) = 0;
                J(0, 2) = -y_hat; // Partial derivatives with respect to x, y, theta
                J(1, 0) = 0;
                J(1, 1) = 1;
                J(1, 2) = x_hat; // Partial derivatives with respect to x, y, theta

                Vector2d e = transformed_point - nearest_target;

                H += J.transpose() * J;
                b += J.transpose() * e;
            }

            // Solve for the perturbation delta_x using H and b
            Vector3d delta_x = H.ldlt().solve(-b); // More stable than H.inverse() * b

            // Update the transformation using the perturbation delta_x
            double delta_theta = delta_x[2];
            Matrix2d delta_R;
            delta_R << std::cos(delta_theta), -std::sin(delta_theta),
                std::sin(delta_theta), std::cos(delta_theta);

            R = delta_R * R;
            t += delta_x.head<2>(); // Update translation, take the first 2 elements of delta_x

            // Compute the mean error
            double mean_error = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();

            // cout << "[INFO] Iteration " << iter + 1 << ": Mean error = " << mean_error << endl;

            // Check for convergence
            if (std::abs(prev_error - mean_error) < tolerance)
            {
                cout << "[INFO] Convergence achieved after " << iter + 1 << " iterations." << endl;
                cout << "[INFO] Mean error = " << mean_error << endl;
                break;
            }

            prev_error = mean_error;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;

        ++num_processed_scans;
        running_average_time += (duration.count() - running_average_time) / num_processed_scans;

        cout << "[INFO] Running average ICP time after " << num_processed_scans << " scan pairs: " << 1.0 / running_average_time << " Hz" << endl;

        // Accumulate the total transformation
        R_total = R * R_total;
        t_total = R * t_total + t;

        // Apply final transformation to the current source scan points
        std::vector<Vector2d> transformed_source_scan;
        for (const auto &point : source_scan)
        {
            transformed_source_scan.push_back(R * point + t);
        }
    }

    // Output the final accumulated transformation
    cout << "[INFO] Final accumulated transformation:" << endl;
    cout << "R_total = \n"
         << R_total << endl;
    cout << "t_total = \n"
         << t_total.transpose() << endl;

    return 0;
}

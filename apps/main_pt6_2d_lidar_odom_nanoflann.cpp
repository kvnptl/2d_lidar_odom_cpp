#include "dataloader/dataloader.hpp"
#include "viewer/viewer.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <nanoflann.hpp>
#include <numeric>
#include <vector>

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using std::cout;
using std::endl;

// Point cloud struct compatible with nanoflann
struct PointCloud
{
    std::vector<Vector2d> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the distance between the vector "p1[0],p1[1],..." and the data point with index "idx_p2" stored in the class:
    inline double kdtree_distance(const double *p1, const size_t idx_p2, size_t /*size*/) const
    {
        const double d0 = p1[0] - pts[idx_p2][0];
        const double d1 = p1[1] - pts[idx_p2][1];
        return d0 * d0 + d1 * d1;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline double kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim == 0)
            return pts[idx][0];
        else
            return pts[idx][1];
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it
    //   more efficiently. This is used to speed up the KD-Tree construction.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
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

        // Initialize point cloud structure for nanoflann
        PointCloud cloud;
        cloud.pts = target_scan;

        // Build KD-Tree
        using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<double, PointCloud>,
            PointCloud,
            2 /* dimensions */>;
        KDTree tree(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        tree.buildIndex();

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

                // Find the nearest neighbor using nanoflann
                size_t nearest_index;
                double nearest_dist_sq;
                nanoflann::KNNResultSet<double> resultSet(1);
                resultSet.init(&nearest_index, &nearest_dist_sq);
                double query_pt[2] = {transformed_point[0], transformed_point[1]};
                tree.findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));

                Vector2d nearest_target = target_scan[nearest_index];

                double error = (transformed_point - nearest_target).norm();
                errors.push_back(error);

                Vector2d transformed_source = R * source_point + t;
                double x_hat = transformed_source[0];
                double y_hat = transformed_source[1];

                Matrix<double, 2, 3> J;
                J(0, 0) = 1;
                J(0, 1) = 0;
                J(0, 2) = -y_hat;
                J(1, 0) = 0;
                J(1, 1) = 1;
                J(1, 2) = x_hat;

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

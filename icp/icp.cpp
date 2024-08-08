#include "icp.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

using std::cout;
using std::endl;

namespace icp
{
    void runICP(
        const std::vector<Eigen::Vector2d> &source_scan,
        const std::vector<Eigen::Vector2d> &target_scan,
        Eigen::Matrix2d &R,
        Eigen::Vector2d &t)
    {
        kdtree::KDTree tree(target_scan);

        const int max_iterations = 50;
        const double tolerance = 1e-4;
        double prev_error = std::numeric_limits<double>::max();

        for (int iter = 0; iter < max_iterations; iter++)
        {
            std::vector<double> errors;
            std::vector<Eigen::Matrix<double, 2, 3>> jacobians;
            Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
            Eigen::Vector3d b = Eigen::Vector3d::Zero();

            for (const auto &source_point : source_scan)
            {
                Eigen::Vector2d transformed_point = R * source_point + t;
                Eigen::Vector2d nearest_target = tree.findNearest(transformed_point);

                // Error for convergence check
                double error = (transformed_point - nearest_target).norm();
                errors.emplace_back(error);

                double x_hat = transformed_point[0];
                double y_hat = transformed_point[1];

                // Jacobian
                Eigen::Matrix<double, 2, 3> J;
                J(0, 0) = 1;
                J(0, 1) = 0;
                J(0, 2) = -y_hat;
                J(1, 0) = 0;
                J(1, 1) = 1;
                J(1, 2) = x_hat;

                // Error for optimization
                Eigen::Vector2d e = transformed_point - nearest_target;

                // Hessian and error vector
                H += J.transpose() * J;
                b += J.transpose() * e;
            }

            // Solve linear system
            const Eigen::Vector3d delta_x = H.ldlt().solve(-b); // More stable than H.inverse() * b

            const double delta_theta = delta_x[2];
            Eigen::Matrix2d delta_R;
            delta_R << std::cos(delta_theta), -std::sin(delta_theta),
                std::sin(delta_theta), std::cos(delta_theta);

            R = delta_R * R;
            t += delta_x.head<2>();

            double mean_error = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();

            if (std::abs(prev_error - mean_error) < tolerance)
            {
                // cout << "[INFO] Convergence achieved after " << iter + 1 << " iterations." << endl;
                // cout << "[INFO] Mean error = " << mean_error << endl;
                break;
            }

            prev_error = mean_error;
        }
    }

    // NOTE: This function was written with the help of Large Generative Models.
    struct Vector2iHash
    {
        std::size_t operator()(const Eigen::Vector2i &v) const
        {
            std::hash<int> hasher;
            return hasher(v.x()) ^ hasher(v.y());
        }
    };

    std::vector<Eigen::Vector2d> downsample(const std::vector<Eigen::Vector2d> &points, const double voxel_size)
    {
        std::unordered_map<Eigen::Vector2i, std::vector<Eigen::Vector2d>, Vector2iHash> voxel_map;

        for (const auto &point : points)
        {
            Eigen::Vector2i voxel_index(
                std::floor(point.x() / voxel_size),
                std::floor(point.y() / voxel_size));

            voxel_map[voxel_index].emplace_back(point);
        }

        std::vector<Eigen::Vector2d> downsampled_points;
        downsampled_points.reserve(voxel_map.size());
        for (const auto &voxel : voxel_map)
        {
            const auto &points_in_voxel = voxel.second;
            Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
            for (const auto &point : points_in_voxel)
            {
                centroid += point;
            }
            centroid /= static_cast<double>(points_in_voxel.size());
            downsampled_points.emplace_back(centroid);
        }

        downsampled_points.shrink_to_fit();

        return downsampled_points;
    }
} // namespace icp
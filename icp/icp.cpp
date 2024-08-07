#include "icp.hpp"

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

void computeICP(const std::vector<Vector2d> &target_scan, const std::vector<Vector2d> &source_scan,
                Matrix2d &R_total, Vector2d &t_total, double &running_average_time, int &num_processed_scans)
{
    KDTree tree(target_scan);

    Matrix2d R = Matrix2d::Identity();
    Vector2d t = Vector2d::Zero();

    const int max_iterations = 100;
    const double tolerance = 1e-3;
    double prev_error = std::numeric_limits<double>::max();

    auto start_time = std::chrono::high_resolution_clock::now();

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        std::vector<double> errors;
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
            J(0, 2) = -y_hat;
            J(1, 0) = 0;
            J(1, 1) = 1;
            J(1, 2) = x_hat;

            Vector2d e = transformed_point - nearest_target;

            H += J.transpose() * J;
            b += J.transpose() * e;
        }

        Vector3d delta_x = H.ldlt().solve(-b);

        double delta_theta = delta_x[2];
        Matrix2d delta_R;
        delta_R << std::cos(delta_theta), -std::sin(delta_theta),
            std::sin(delta_theta), std::cos(delta_theta);

        R = delta_R * R;
        t += delta_x.head<2>();

        double mean_error = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();

        if (std::abs(prev_error - mean_error) < tolerance)
        {
            std::cout << "[INFO] Convergence achieved after " << iter + 1 << " iterations." << std::endl;
            std::cout << "[INFO] Mean error = " << mean_error << std::endl;
            break;
        }

        prev_error = mean_error;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;

    ++num_processed_scans;
    running_average_time += (duration.count() - running_average_time) / num_processed_scans;

    std::cout << "[INFO] Running average ICP time after " << num_processed_scans << " scan pairs: " << 1.0 / running_average_time << " Hz" << std::endl;

    R_total = R * R_total;
    t_total = R * t_total + t;
}

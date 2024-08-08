#pragma once
#include <Eigen/Core>
#include <open3d/Open3D.h>
#include <vector>

namespace viewer
{
    void viewCloud(const std::vector<Eigen::Vector2d> &pcd);

    void viewCloud(const std::vector<Eigen::Vector2d> &pcd, open3d::visualization::Visualizer &visualizer);

    void viewTwoClouds(const std::vector<Eigen::Vector2d> &pcd1, const std::vector<Eigen::Vector2d> &pcd2);

    void viewTwoClouds(const std::vector<Eigen::Vector2d> &pcd1, const std::vector<Eigen::Vector2d> &pcd2, open3d::visualization::Visualizer &visualizer);
} // namespace viewer
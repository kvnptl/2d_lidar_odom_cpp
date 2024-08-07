#pragma once
#include <Eigen/Core>
#include <vector>

namespace viewer
{
    void viewCloud(const std::vector<Eigen::Vector2d> &pcd);

    void viewTwoClouds(const std::vector<Eigen::Vector2d> &pcd1, const std::vector<Eigen::Vector2d> &pcd2);
} // namespace viewer
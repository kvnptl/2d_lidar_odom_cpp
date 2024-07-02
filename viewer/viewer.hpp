#pragma once
#include <Eigen/Core>
#include <vector>

namespace viewer
{
    void viewCloud(const std::vector<Eigen::Vector2d> &pcd);
} // namespace viewer
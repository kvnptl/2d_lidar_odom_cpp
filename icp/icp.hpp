#pragma once

#include "kdtree.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

void runICP(
    const std::vector<Eigen::Vector2d> &source_scan,
    const std::vector<Eigen::Vector2d> &target_scan,
    Eigen::Matrix2d &R,
    Eigen::Vector2d &t);

std::vector<Eigen::Vector2d> voxelGridDownsample(const std::vector<Eigen::Vector2d> &points, double voxel_size);

#pragma once

#include "kdtree.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <numeric>
#include <chrono>
#include <iostream>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

void computeICP(const std::vector<Vector2d> &target_scan, const std::vector<Vector2d> &source_scan,
                Matrix2d &R_total, Vector2d &t_total, double &running_average_time, int &num_processed_scans);

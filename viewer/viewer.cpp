#include "viewer.hpp"
#include <algorithm>
#include <iostream>
#include <open3d/Open3D.h>
#include <open3d/core/ShapeUtil.h>
#include <utility>
#include <vector>

namespace viewer
{
  void viewCloud(const std::vector<Eigen::Vector2d> &pcd)
  {
    std::vector<Eigen::Vector3d> pts(pcd.size());
    std::transform(pcd.cbegin(), pcd.cend(), pts.begin(), [](const auto &p)
                   { return Eigen::Vector3d(p.x(), p.y(), 0.0); });
    open3d::geometry::PointCloud pointcloud{pts};
    pointcloud.PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
    open3d::visualization::DrawGeometries(
        {std::make_shared<open3d::geometry::PointCloud>(pointcloud)});
  }

  void viewTwoClouds(const std::vector<Eigen::Vector2d> &pcd1, const std::vector<Eigen::Vector2d> &pcd2)
  {
    std::vector<Eigen::Vector3d> pts1(pcd1.size());
    std::transform(pcd1.cbegin(), pcd1.cend(), pts1.begin(), [](const auto &p)
                   { return Eigen::Vector3d(p.x(), p.y(), 0.0); });

    std::vector<Eigen::Vector3d> pts2(pcd2.size());
    std::transform(pcd2.cbegin(), pcd2.cend(), pts2.begin(), [](const auto &p)
                   { return Eigen::Vector3d(p.x(), p.y(), 0.0); });

    open3d::geometry::PointCloud pointcloud1{pts1};
    pointcloud1.PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0)); // Red color

    open3d::geometry::PointCloud pointcloud2{pts2};
    pointcloud2.PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0)); // Green color

    open3d::visualization::DrawGeometries(
        {std::make_shared<open3d::geometry::PointCloud>(pointcloud1),
         std::make_shared<open3d::geometry::PointCloud>(pointcloud2)});
  }
} // namespace viewer
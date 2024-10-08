#include "viewer.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <open3d/Open3D.h>
#include <open3d/core/ShapeUtil.h>
#include <thread>
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

  // To view a point cloud continuously with a visualizer
  // NOTE: This function was written with the help of Large Generative Models.
  void viewCloud(const std::vector<Eigen::Vector2d> &pcd, open3d::visualization::Visualizer &visualizer)
  {
    std::vector<Eigen::Vector3d> pts(pcd.size());
    std::transform(pcd.cbegin(), pcd.cend(), pts.begin(), [](const auto &p)
                   { return Eigen::Vector3d(p.x(), p.y(), 0.0); });

    auto pointcloud = std::make_shared<open3d::geometry::PointCloud>(pts);
    pointcloud->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0)); // Red color

    // Clear the visualizer and add new point cloud
    visualizer.ClearGeometries();
    visualizer.AddGeometry(pointcloud);

    // Update the visualizer
    visualizer.UpdateGeometry();
    visualizer.PollEvents();
    visualizer.UpdateRender();
  }

  // Visualize two point clouds
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

  // To view two point clouds continuously with a visualizer
  // NOTE: This function was written with the help of Large Generative Models.
  void viewTwoClouds(const std::vector<Eigen::Vector2d> &pcd1, const std::vector<Eigen::Vector2d> &pcd2, open3d::visualization::Visualizer &visualizer)
  {
    std::vector<Eigen::Vector3d> pts1(pcd1.size());
    std::transform(pcd1.cbegin(), pcd1.cend(), pts1.begin(), [](const auto &p)
                   { return Eigen::Vector3d(p.x(), p.y(), 0.0); });

    std::vector<Eigen::Vector3d> pts2(pcd2.size());
    std::transform(pcd2.cbegin(), pcd2.cend(), pts2.begin(), [](const auto &p)
                   { return Eigen::Vector3d(p.x(), p.y(), 0.0); });

    auto pointcloud1 = std::make_shared<open3d::geometry::PointCloud>(pts1);
    pointcloud1->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0)); // Red color

    auto pointcloud2 = std::make_shared<open3d::geometry::PointCloud>(pts2);
    pointcloud2->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0)); // Green color

    // Clear the visualizer and add new point clouds
    visualizer.ClearGeometries();
    visualizer.AddGeometry(pointcloud1);
    visualizer.AddGeometry(pointcloud2);

    // Update the visualizer
    visualizer.UpdateGeometry();
    visualizer.PollEvents();
    visualizer.UpdateRender();
  }
} // namespace viewer
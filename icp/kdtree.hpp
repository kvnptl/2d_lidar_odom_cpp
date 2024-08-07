#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

using Eigen::Vector2d;

class KDNode
{
public:
    Vector2d point;
    KDNode *left;
    KDNode *right;

    KDNode(const Vector2d &pt);
    ~KDNode();
};

class KDTree
{
private:
    KDNode *root;

    KDNode *build(std::vector<Vector2d>::iterator start, std::vector<Vector2d>::iterator end, int depth);
    void nearest(KDNode *node, const Vector2d &target, KDNode *&best, double &bestDist, int depth);

public:
    KDTree(const std::vector<Vector2d> &points);
    ~KDTree();
    Vector2d findNearest(const Vector2d &target);
};
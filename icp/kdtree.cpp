#include "kdtree.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

KDNode::KDNode(const Vector2d &pt) : point(pt), left(nullptr), right(nullptr) {}

KDNode::~KDNode()
{
    delete left;
    delete right;
}

KDTree::KDTree(const std::vector<Vector2d> &points)
{
    std::vector<Vector2d> mutable_points = points;
    root = build(mutable_points.begin(), mutable_points.end(), 0);
}

KDTree::~KDTree()
{
    delete root;
}

KDNode *KDTree::build(std::vector<Vector2d>::iterator start, std::vector<Vector2d>::iterator end, int depth)
{
    if (start >= end)
        return nullptr;

    int axis = depth % 2;
    std::sort(start, end, [axis](const Vector2d &a, const Vector2d &b)
              { return a[axis] < b[axis]; });

    auto mid = start + (end - start) / 2;
    KDNode *node = new KDNode(*mid);
    node->left = build(start, mid, depth + 1);
    node->right = build(mid + 1, end, depth + 1);

    return node;
}

void KDTree::nearest(KDNode *node, const Vector2d &target, KDNode *&best, double &bestDist, int depth)
{
    if (!node)
        return;

    double dist = (node->point - target).squaredNorm();
    if (dist < bestDist)
    {
        bestDist = dist;
        best = node;
    }

    int axis = depth % 2;
    KDNode *near = target[axis] < node->point[axis] ? node->left : node->right;
    KDNode *far = target[axis] < node->point[axis] ? node->right : node->left;

    nearest(near, target, best, bestDist, depth + 1);

    double dx = target[axis] - node->point[axis];
    if (dx * dx < bestDist)
    {
        nearest(far, target, best, bestDist, depth + 1);
    }
}

Vector2d KDTree::findNearest(const Vector2d &target)
{
    KDNode *best = nullptr;
    double bestDist = std::numeric_limits<double>::max();
    nearest(root, target, best, bestDist, 0);
    return best ? best->point : Vector2d();
}
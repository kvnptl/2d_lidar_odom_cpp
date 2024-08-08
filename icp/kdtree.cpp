/*
NOTE: This code was written with the help of Large Generative Models.
*/

#include "kdtree.hpp"
#include <algorithm>
#include <limits>

namespace kdtree
{
    KDNode::KDNode(const Eigen::Vector2d &pt) : point(pt), left(nullptr), right(nullptr) {}
    KDNode::~KDNode()
    {
        delete left;
        delete right;
    }

    // Recursive function to build the KDTree
    KDNode *KDTree::build(std::vector<Eigen::Vector2d>::iterator start, std::vector<Eigen::Vector2d>::iterator end, int depth)
    {
        if (start >= end)
            return nullptr;

        int axis = depth % 2;
        // Sort points based on the current axis
        std::sort(start, end, [axis](const Eigen::Vector2d &a, const Eigen::Vector2d &b)
                  { return a[axis] < b[axis]; });

        // Find the median point
        auto mid = start + (end - start) / 2;
        KDNode *node = new KDNode(*mid);
        // Recursively build the left and right subtrees
        node->left = build(start, mid, depth + 1);
        node->right = build(mid + 1, end, depth + 1);

        return node;
    }

    // Recursive function to find the nearest neighbor
    void KDTree::nearest(KDNode *node, const Eigen::Vector2d &target, KDNode *&best, double &bestDist, int depth)
    {
        if (!node)
            return;

        // Calculate squared distance from target to current node
        double dist = (node->point - target).squaredNorm();
        if (dist < bestDist)
        {
            bestDist = dist;
            best = node;
        }

        int axis = depth % 2;

        // Determine which subtree to search first
        KDNode *near = target[axis] < node->point[axis] ? node->left : node->right;
        KDNode *far = target[axis] < node->point[axis] ? node->right : node->left;

        // Search the nearer subtree
        nearest(near, target, best, bestDist, depth + 1);

        // Check if we need to search the farther subtree
        double dx = target[axis] - node->point[axis];
        if (dx * dx < bestDist)
        {
            nearest(far, target, best, bestDist, depth + 1);
        }
    }

    KDTree::KDTree(const std::vector<Eigen::Vector2d> &points)
    {
        std::vector<Eigen::Vector2d> mutable_points = points;
        root = build(mutable_points.begin(), mutable_points.end(), 0);
    }

    KDTree::~KDTree()
    {
        delete root;
    }

    // Find the nearest neighbor to a target point
    Eigen::Vector2d KDTree::findNearest(const Eigen::Vector2d &target)
    {
        KDNode *best = nullptr;
        double bestDist = std::numeric_limits<double>::max();
        nearest(root, target, best, bestDist, 0);
        return best ? best->point : Eigen::Vector2d();
    }
} // namespace kdtree
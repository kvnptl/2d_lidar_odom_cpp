#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using Eigen::Vector2d;

// Structure to represent a 2D point
struct Point2D
{
    double x, y;
    Point2D(double _x = 0, double _y = 0) : x(_x), y(_y) {}
};

// Structure to represent a node in the KD-Tree
struct KDNode
{
    Vector2d point;
    KDNode *left, *right;
    KDNode(const Vector2d &pt) : point(pt), left(nullptr), right(nullptr) {}
};

// Helper function to calculate squared distance between two points
double distanceSquared(const Vector2d &a, const Vector2d &b)
{
    return (a - b).squaredNorm();
}

// Function to build the KD-Tree
KDNode *buildKDTree(std::vector<Vector2d> &points, int depth = 0)
{
    if (points.empty())
        return nullptr;

    // Determine axis based on depth
    int axis = depth % 2;

    // Sort points based on the current axis
    std::sort(points.begin(), points.end(), [axis](const Vector2d &a, const Vector2d &b)
              { return a[axis] < b[axis]; });

    // Select the median point
    size_t medianIndex = points.size() / 2;
    Vector2d medianPoint = points[medianIndex];

    // Create the node
    KDNode *node = new KDNode(medianPoint);

    // Recursively build left and right subtrees
    std::vector<Vector2d> leftPoints(points.begin(), points.begin() + medianIndex);
    std::vector<Vector2d> rightPoints(points.begin() + medianIndex + 1, points.end());

    node->left = buildKDTree(leftPoints, depth + 1);
    node->right = buildKDTree(rightPoints, depth + 1);

    return node;
}

// Function to search for the nearest neighbor in the KD-Tree
void nearestNeighborSearch(KDNode *root, const Vector2d &target, KDNode *&best, double &bestDist, int depth = 0)
{
    if (!root)
        return;

    // Calculate the distance from the target to the current node
    double d = distanceSquared(root->point, target);
    if (d < bestDist)
    {
        bestDist = d;
        best = root;
    }

    // Determine axis based on depth
    int axis = depth % 2;
    KDNode *nextBranch = nullptr;
    KDNode *oppositeBranch = nullptr;

    // Decide which branch to explore next
    if (target[axis] < root->point[axis])
    {
        nextBranch = root->left;
        oppositeBranch = root->right;
    }
    else
    {
        nextBranch = root->right;
        oppositeBranch = root->left;
    }

    // Recursively search the next branch
    nearestNeighborSearch(nextBranch, target, best, bestDist, depth + 1);

    // Check if we need to search the opposite branch
    double radius = std::abs(target[axis] - root->point[axis]);
    if (radius * radius < bestDist)
    {
        nearestNeighborSearch(oppositeBranch, target, best, bestDist, depth + 1);
    }
}

int main()
{
    // Example point cloud
    std::vector<Vector2d> points = {{3, 6}, {17, 15}, {11, 9}, {13, 15}, {6, 12}, {9, 1}, {2, 7}, {10, 19}};

    // Build KD-Tree
    KDNode *kdTreeRoot = buildKDTree(points);

    // Query point
    Vector2d query(10, 10);

    // Nearest neighbor search
    KDNode *best = nullptr;
    double bestDist = std::numeric_limits<double>::max();
    nearestNeighborSearch(kdTreeRoot, query, best, bestDist);

    // Print the result
    if (best)
    {
        std::cout << "The nearest neighbor to (" << query.x() << ", " << query.y() << ") is ("
                  << best->point.x() << ", " << best->point.y() << ") with squared distance " << bestDist << std::endl;
    }
    else
    {
        std::cout << "No nearest neighbor found!" << std::endl;
    }

    return 0;
}

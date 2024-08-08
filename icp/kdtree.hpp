#include <Eigen/Core>
#include <vector>

namespace kdtree
{
    class KDNode
    {
    public:
        Eigen::Vector2d point; // Point stored in the node
        KDNode *left;          // Pointer to the left child
        KDNode *right;         // Pointer to the right child

        KDNode(const Eigen::Vector2d &pt);
        ~KDNode();
    };

    class KDTree
    {
    private:
        KDNode *root; // Root node of the KDTree

        // Recursive function to build the KDTree
        KDNode *build(std::vector<Eigen::Vector2d>::iterator start, std::vector<Eigen::Vector2d>::iterator end, int depth);

        // Recursive function to find the nearest neighbor
        void nearest(KDNode *node, const Eigen::Vector2d &target, KDNode *&best, double &bestDist, int depth);

    public:
        KDTree(const std::vector<Eigen::Vector2d> &points);
        ~KDTree();

        // Find the nearest neighbor to a target point
        Eigen::Vector2d findNearest(const Eigen::Vector2d &target);
    };

} // namespace kdtree
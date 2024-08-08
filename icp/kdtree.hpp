#include <Eigen/Core>
#include <vector>

class KDNode
{
public:
    Eigen::Vector2d point;
    KDNode *left;
    KDNode *right;

    KDNode(const Eigen::Vector2d &pt);
    ~KDNode();
};

class KDTree
{
private:
    KDNode *root;
    KDNode *build(std::vector<Eigen::Vector2d>::iterator start, std::vector<Eigen::Vector2d>::iterator end, int depth);
    void nearest(KDNode *node, const Eigen::Vector2d &target, KDNode *&best, double &bestDist, int depth);

public:
    KDTree(const std::vector<Eigen::Vector2d> &points);
    ~KDTree();
    Eigen::Vector2d findNearest(const Eigen::Vector2d &target);
};
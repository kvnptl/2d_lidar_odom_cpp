#include <Eigen/Core>
#include <iostream>
#include <limits>
#include <vector>

// Define a type alias for convenience
using PointCloud = std::vector<Eigen::Vector2d>;

// Function to find the nearest neighbor correspondences
std::vector<std::pair<int, int>> findCorrespondences(const PointCloud &scan1, const PointCloud &scan2)
{
    std::vector<std::pair<int, int>> correspondences;

    for (int i = 0; i < scan1.size(); ++i)
    {
        double min_distance = std::numeric_limits<double>::max();
        int best_match = -1;

        for (int j = 0; j < scan2.size(); ++j)
        {
            double distance = (scan1[i] - scan2[j]).squaredNorm();
            if (distance < min_distance)
            {
                min_distance = distance;
                best_match = j;
            }
        }

        if (best_match != -1)
        {
            correspondences.emplace_back(i, best_match);
        }
    }

    return correspondences;
}

int main()
{
    // Example point clouds (replace with actual data)
    PointCloud scan1 = {
        {0.0, 0.0},
        {1.0, 1.0},
        {2.0, 2.0}};

    PointCloud scan2 = {
        {2.1, 2.1},
        {0.1, 0.1},
        {1.1, 1.1},
    };

    // Find correspondences
    auto correspondences = findCorrespondences(scan1, scan2);

    // Print correspondences
    for (const auto &correspondence : correspondences)
    {
        std::cout << "Scan1 point " << correspondence.first << " corresponds to Scan2 point " << correspondence.second << std::endl;
    }

    return 0;
}
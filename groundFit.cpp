#include "groundFit.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <random>
#include <limits>

PlaneFitResults fitGroundPlaneRansac(
    const std::vector<Eigen::Vector3f> &points,
    int max_iterations,
    float distance_threshold,
    double uprightDotThreshold)
    {
        PlaneFitResults best_fit;
        best_fit.plane.normal = Eigen::Vector3f::Zero();
        best_fit.plane.d = 0.0;
        size_t best_inlier_count = 0;

        if(points.size() < 3) {
            std::cerr << "Not enough points to fit a plane." << std::endl;
            return best_fit;
        }


        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

        for (int it =0; it < max_iterations; ++it) {
            // Randomly sample 3 points
            size_t idx1 = dist(rng);
            size_t idx2 = dist(rng);
            size_t idx3 = dist(rng);

            if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) {
                --it; // Resample if indices are not unique
                continue;
            }

            Eigen::Vector3f p1 = points[idx1];
            Eigen::Vector3f p2 = points[idx2];
            Eigen::Vector3f p3 = points[idx3];

            // Compute plane normal
            Eigen::Vector3f normal = (p2 - p1).cross(p3 - p1).normalized();
            float d = -normal.dot(p1);

            // Check upright constraint
            if (std::abs(normal.dot(Eigen::Vector3f::UnitZ())) < uprightDotThreshold) {
                continue; // Skip if not upright enough
            }

            // Count inliers
            std::vector<int> inlier_indices;
            for (size_t i = 0; i < points.size(); ++i) {
                float distance = std::abs(normal.dot(points[i]) + d);
                if (distance < distance_threshold) {
                    inlier_indices.push_back(static_cast<int>(i));
                }
            }

            // Update best fit if current one is better
            if (inlier_indices.size() > best_inlier_count) {
                best_inlier_count = inlier_indices.size();
                best_fit.plane.normal = normal;
                best_fit.plane.d = d;
                best_fit.inlier_indices = inlier_indices;
            }
        }


        return best_fit;
    }


std::vector<Eigen::Vector3f> getGroundPoints(
    const std::vector<Eigen::Vector3f> &points,
    const std::vector<int>& inliers
)
{
    std::vector<Eigen::Vector3f> ground_points;
    for (const auto& idx : inliers) {
        ground_points.push_back(points[idx]);
    }
    return ground_points;
}

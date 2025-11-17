#ifndef GROUND_FIT_HPP
#define GROUND_FIT_HPP

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <limits>

struct PlaneModel {
    Eigen::Vector3f normal; // plane normal
    float d; // plane offset
};

struct PlaneFitResults {
    PlaneModel plane;
    std::vector<int> inlier_indices;
};


PlaneFitResults fitGroundPlaneRansac(
    const std::vector<Eigen::Vector3f> &points,
    int max_iterations,
    float distance_threshold,
    double uprightDotThreshold
);

PlaneModel refinePlaneLeastSquares(
    const std::vector<Eigen::Vector3f> &points,
    const std::vector<int> &inlier_indices
);

std::vector<Eigen::Vector3f> getGroundPoints(
    const std::vector<Eigen::Vector3f> &points,
    const std::vector<int>& inliers
);

#endif // GROUND_FIT_HPP

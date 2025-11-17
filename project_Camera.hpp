#ifndef PROJECT_CAMERA_HPP
#define PROJECT_CAMERA_HPP

#include "groundFit.hpp"
#include "parse_json.hpp"
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <limits>

struct Intrinsic;
struct Extrinsic;

std::vector<Eigen::Vector3f> project_Camera_To_GroundPoints(
    const Extrinsic &Camera_Extrinsic,
    const Intrinsic &Camera_Intrinsic,
    const PlaneModel &ground_plane
);

std::vector<Eigen::Vector3f> generate_Ground_Polygon(
    std::vector<Eigen::Vector3f> &ground_corners,
    float point_spacing
);

#endif // PROJECT_CAMERA_HPP

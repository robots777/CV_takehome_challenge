#ifndef PARSE_JSON_HPP
#define PARSE_JSON_HPP

#include <iostream>
#include <fstream>
#include <vector>

#include "Eigen/Dense"
#include "nlohmann/json.hpp"

#include "ply_stream.hpp"
#include "groundFit.hpp"
#include "project_Camera.hpp"

// this is a pinhole camera model, with zero distortion or skew
struct Intrinsic {
    Eigen::Matrix3f K; // intrinsic matrix K
    int rows; // height of the image
    int cols; // width of the image
};

Intrinsic ExtractIntrinsicFromJson(const nlohmann::json &intrinsic_json);

struct Extrinsic {
    Eigen::Matrix3f R; // rotation matrix
    Eigen::Vector3f c; // camera center
};

Extrinsic ExtractExtrinsicFromJson(const nlohmann::json &extrinsic_json);

Eigen::Vector3f ExtractPointFromJson(const nlohmann::json &point_json);

void ParseSceneDataJson(
    Intrinsic &intrinsic, // output
    std::vector<Extrinsic> &extrinsics, // output
    std::vector<Eigen::Vector3f> &points, // output
    const std::string &json_file_path // input
);

#endif // PARSE_JSON_HPP

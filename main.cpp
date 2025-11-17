#include <iostream>
#include <fstream>
#include <vector>

#include "Eigen/Dense"
#include "nlohmann/json.hpp"

#include "ply_stream.hpp"
#include "groundFit.hpp"
#include "project_Camera.hpp"
#include "parse_json.hpp"

//Move to parse_json.hpp
//This the Ransac solution 

int main(int, char**){

    // read in the scene data:
    const std::string json_file_path = "./scene_data.json";
    Intrinsic intrinsic;
    std::vector<Extrinsic> extrinsics;
    std::vector<Eigen::Vector3f> points;
    ParseSceneDataJson(intrinsic, extrinsics, points, json_file_path);


    // write results to this folder:
    const std::string output_folder = "./output/";

    PointPlyStream ply_stream(output_folder + "raw_cloud.ply");
    ply_stream.WriteHeader({"float x", "float y", "float z", "uchar red", "uchar green", "uchar blue"});
    for (const auto &pt : points) {
         ply_stream << pt.x() << pt.y() << pt.z() << (unsigned char)0 << (unsigned char)255 << (unsigned char)255;
    }

    std::cout << "Number of Frames: " << extrinsics.size() << "\n";
    std::cout << "Wrote point cloud to " << output_folder + "raw_cloud.ply" << "\n";
    
    //Calculate Ransac Plane Fit
    PlaneFitResults plane_fit = fitGroundPlaneRansac(points, 2000, 3.0f, 0.85);

    std::cout << "Fitted Plane Normal: " << plane_fit.plane.normal.transpose() << "\n";
    std::cout << "Fitted Plane d: " << plane_fit.plane.d << "\n";
    std::cout << "Number of Inliers: " << plane_fit.inlier_indices.size() << "\n";

    //Save Ground Plane Points and Building Points to PLY
    
    PointPlyStream ground_ply_stream(output_folder + "Ransac_ground_points.ply");
    ground_ply_stream.WriteHeader({"float x", "float y", "float z", "uchar red", "uchar green", "uchar blue"});
    for (int i =0; i < points.size(); ++i) {
        if(std::find(plane_fit.inlier_indices.begin(), plane_fit.inlier_indices.end(), i) != plane_fit.inlier_indices.end()) {
            const auto &pt = points[i];
            ground_ply_stream << pt.x() << pt.y() << pt.z() << (unsigned char)0 << (unsigned char)255 << (unsigned char)0;
        }
        else {
            const auto &pt = points[i];
            ground_ply_stream << pt.x() << pt.y() << pt.z() << (unsigned char)255 << (unsigned char)0 << (unsigned char)0;
        }
    }
    std::cout << "Wrote ground and building points to " << output_folder + "Ransac_ground_points.ply" << "\n";


    //Camera Projections to Ground Plane
    PointPlyStream camera_ground_stream(output_folder + "Ransac_camera_ground_points.ply");
    camera_ground_stream.WriteHeader({"float x", "float y", "float z", "uchar red", "uchar green", "uchar blue"});
    for (size_t frame_idx = 0; frame_idx < extrinsics.size(); ++frame_idx) {

        std::vector<Eigen::Vector3f> ground_corners = project_Camera_To_GroundPoints(extrinsics[frame_idx], intrinsic, plane_fit.plane);
        std::vector<Eigen::Vector3f> ground_points  = generate_Ground_Polygon(ground_corners, 2.0f);
        for (const auto &pt : ground_points) {
            camera_ground_stream << pt.x() << pt.y() << pt.z() << (unsigned char)153 << (unsigned char)134 << (unsigned char)127;
        }
    }
    std::cout << "Wrote ground camera " << output_folder + "Ransac_camera_ground_points.ply" << "\n";
    return 0;
}

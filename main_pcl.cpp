#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>

#include "Eigen/Dense"
#include "nlohmann/json.hpp"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "ply_stream.hpp"

#include "groundFit.hpp"
#include "project_Camera.hpp"
#include "parse_json.hpp"

/* I decided to solve this problem both using Point Cloud Library, and using RANSAC approach.  This is the PCL solution.
*/

int main(int, char**){

    // read in the scene data:
    const std::string json_file_path = "./scene_data.json";
    Intrinsic intrinsic;
    std::vector<Extrinsic> extrinsics;
    std::vector<Eigen::Vector3f> points;
    ParseSceneDataJson(intrinsic, extrinsics, points, json_file_path);


    // write results to this folder:
    const std::string output_folder = "./output/";

    //Write Raw data
    PointPlyStream ply_stream(output_folder + "pcl_raw_cloud.ply");
    ply_stream.WriteHeader({"float x", "float y", "float z", "uchar red", "uchar green", "uchar blue"});
    for (const auto &pt : points) {
         ply_stream << pt.x() << pt.y() << pt.z() << (unsigned char)0 << (unsigned char)255 << (unsigned char)255;
    }
    std::cout << "Wrote raw point cloud to " << output_folder + "pcl_raw_cloud.ply" << "\n";
    
    //Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < points.size(); ++i) {
        cloud->points[i].x = points[i].x();
        cloud->points[i].y = points[i].y();
        cloud->points[i].z = points[i].z();
    }

    //pcl::io::savePCDFileBinary (output_folder + "pt_cloud_binary.pcd", *cloud);

    //SAC Segmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (3.0f);
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    seg.setInputCloud (cloud);
    seg.segment (inliers, coefficients);
    if (inliers.indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
    std::cout << "Plane coefficients: " << coefficients.values[0] << ","
                                      << coefficients.values[1] << ","
                                      << coefficients.values[2] << ","
                                      << coefficients.values[3] << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colorized(new pcl::PointCloud<pcl::PointXYZRGB>);

    //Fast Lookup used to colorize the data
    std::unordered_set<int> inlier_set(inliers.indices.begin(),
                                   inliers.indices.end());

    //Colorized Point Cloud
    for(size_t i =0; i <cloud->points.size(); ++i)
    {
        pcl::PointXYZRGB p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;

        if(inlier_set.count(i)){
            //Inlier
            p.r = 0;
            p.g = 255;
            p.b = 0;
        }
        else{
            //Outliers
            p.r = 255;
            p.g = 0;
            p.b = 0;
        }
        cloud_colorized->points.push_back(p);
    }

    cloud_colorized->width =cloud->width;
    cloud_colorized->height =cloud->height;
    cloud_colorized->is_dense =cloud->is_dense;

    pcl::io::savePLYFile (output_folder + "pcl_ground.ply", *cloud_colorized);
    std::cout << "Write ground plane to " << output_folder + "pcl_ground.ply" <<std::endl;

    //Project Camera Ground Plane
    PlaneModel ground_plane;
    ground_plane.normal[0] = coefficients.values[0];
    ground_plane.normal[1] = coefficients.values[1];
    ground_plane.normal[2] = coefficients.values[2];
    ground_plane.d = coefficients.values[3];

    //Camera Projections to Ground Plane
    PointPlyStream camera_ground_stream(output_folder + "pcl_camera_ground_points.ply");
    camera_ground_stream.WriteHeader({"float x", "float y", "float z", "uchar red", "uchar green", "uchar blue"});
    for (size_t frame_idx = 0; frame_idx < extrinsics.size(); ++frame_idx) {

        std::vector<Eigen::Vector3f> ground_corners = project_Camera_To_GroundPoints(extrinsics[frame_idx], intrinsic, ground_plane);
        std::vector<Eigen::Vector3f> ground_points  = generate_Ground_Polygon(ground_corners, 2.0f);
        for (const auto &pt : ground_points) {
            camera_ground_stream << pt.x() << pt.y() << pt.z() << (unsigned char)153 << (unsigned char)134 << (unsigned char)127;
        }
    }
    std::cout << "Write camera ground to " << output_folder + "pcl_camera_ground_points.ply" <<std::endl;
    return 0;
}

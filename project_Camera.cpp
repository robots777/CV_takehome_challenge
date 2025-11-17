#include "project_Camera.hpp"
#include "groundFit.hpp"



std::vector<Eigen::Vector3f> project_Camera_To_GroundPoints(
    const Extrinsic &Camera_Extrinsic,
    const Intrinsic &Camera_Intrinsic,
    const PlaneModel &ground_plane
) {
    std::vector<Eigen::Vector3f> ground_points;
    Eigen::Vector3f n = ground_plane.normal;
    float d = ground_plane.d;

    //Image corners in pixel coordinates
    std::vector<Eigen::Vector2f> image_corners = {
        /*
        Eigen::Vector2f(0, 0),
        Eigen::Vector2f(Camera_Intrinsic.cols - 1, 0),
        Eigen::Vector2f(0, Camera_Intrinsic.rows - 1),
        Eigen::Vector2f(Camera_Intrinsic.cols - 1, Camera_Intrinsic.rows - 1) */
        Eigen::Vector2f(0,Camera_Intrinsic.rows-1),
        Eigen::Vector2f(Camera_Intrinsic.cols-1, Camera_Intrinsic.rows-1),
        Eigen::Vector2f(Camera_Intrinsic.cols-1, 0),
        Eigen::Vector2f(0,0)
    };

    for (const auto &corner : image_corners) {
        //Backproject to 3D ray in camera coordinates
        Eigen::Vector3f ray_dir_camera;
        ray_dir_camera << 
            (corner.x() - Camera_Intrinsic.K(0,2)) / Camera_Intrinsic.K(0,0),
            (corner.y() - Camera_Intrinsic.K(1,2)) / Camera_Intrinsic.K(1,1),
            1.0f;
        ray_dir_camera.normalize();


        //Transform ray to world coordinates
        Eigen::Vector3f ray_dir_world = Camera_Extrinsic.R.inverse() * ray_dir_camera;
        Eigen::Vector3f camera_center = Camera_Extrinsic.c;

        //Compute intersection with ground plane
        float t = -(n.dot(camera_center) + d) / n.dot(ray_dir_world);
        Eigen::Vector3f ground_point = camera_center + t * ray_dir_world;
        ground_points.push_back(ground_point);
    }
    return ground_points;

}    

std::vector<Eigen::Vector3f> generate_Ground_Polygon(
    std::vector<Eigen::Vector3f> &ground_corners,
    float point_spacing
)
{
    std::vector<Eigen::Vector3f> ground_polygon_points;

    //Ground_corners : bottom-left, bottom-right, top-right, top-left
    for (size_t i = 0; i < ground_corners.size(); ++i) {
        Eigen::Vector3f start = ground_corners[i];
        Eigen::Vector3f end = ground_corners[(i + 1) % ground_corners.size()];
        Eigen::Vector3f edge = end - start;
        float edge_length = edge.norm();
        Eigen::Vector3f edge_dir = edge.normalized();

        int num_points = static_cast<int>(edge_length / point_spacing);
        for (int j = 0; j <= num_points; ++j) {
            Eigen::Vector3f point = start + j * point_spacing * edge_dir;
            ground_polygon_points.push_back(point);
        }
    }

    return ground_polygon_points;
}

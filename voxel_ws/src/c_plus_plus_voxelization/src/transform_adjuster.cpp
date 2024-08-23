#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
#include "projected_point.h"


// Function to get the initial camera to base_link transform (for reference)
Eigen::Matrix4f getInitialCameraToBaseLinkTransform() {
    Eigen::Vector3d translation(0.039145, 0.3005, -0.021825);
    Eigen::Quaterniond quaternion(0.50995227, -0.47273134, 0.49787735, -0.51825854);
    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation_matrix.cast<float>();
    transform.block<3, 1>(0, 3) = translation.cast<float>();

    return transform;
}

// Function to project LiDAR points onto the image using the current transform
void projectLiDARPoints(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud,
                        const cv::Mat& image, 
                        const cv::Mat& intrinsic_matrix, 
                        const Eigen::Matrix4f& camera_to_base_link,
                        std::vector<ProjectedPoint>& projected_points) {  // Reference to projected_points

    cv::Mat image_with_points = image.clone();

    std::vector<Eigen::Vector3d> points_in_camera;

    for (const auto& point : pcl_cloud.points) {
        Eigen::Vector4f point_base_link(point.x, point.y, point.z, 1.0);
        Eigen::Vector4f point_camera = camera_to_base_link.inverse() * point_base_link;
        points_in_camera.emplace_back(point_camera.x(), point_camera.y(), point_camera.z());
    }

    // Convert to OpenCV points
    std::vector<cv::Point3f> points3D_camera;
    for (const auto& point : points_in_camera) {
        points3D_camera.emplace_back(point.x(), point.y(), point.z());
    }

    // Project points onto the image plane
    std::vector<cv::Point2f> points2D;
    cv::projectPoints(points3D_camera, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), intrinsic_matrix, cv::Mat(), points2D);

    // Populate the vector of ProjectedPoint structures
    for (size_t i = 0; i < points2D.size(); ++i) {
            ProjectedPoint proj_point; // Create an instance of ProjectedPoint
            proj_point.point2D = points2D[i];
            proj_point.point3D = Eigen::Vector3f(points_in_camera[i].x(), points_in_camera[i].y(), points_in_camera[i].z());
            projected_points.push_back(proj_point);  // Push the instance into the vector
    }

    // Determine min and max intensity
    double min_intensity = std::numeric_limits<double>::max();
    double max_intensity = std::numeric_limits<double>::min();

    for (size_t i = 0; i < points_in_camera.size(); ++i) {
        if (points_in_camera[i].z() > 0 && points2D[i].x >= 0 && points2D[i].x < image.cols && points2D[i].y >= 0 && points2D[i].y < image.rows) {
            double intensity = pcl_cloud.points[i].intensity;
            min_intensity = std::min(min_intensity, intensity);
            max_intensity = std::max(max_intensity, intensity);
        }
    }

    double intensity_range = max_intensity - min_intensity;
    if (intensity_range == 0) intensity_range = 1;

    for (size_t i = 0; i < points_in_camera.size(); ++i) {
        if (points_in_camera[i].z() > 0 && points2D[i].x >= 0 && points2D[i].x < image.cols && points2D[i].y >= 0 && points2D[i].y < image.rows) {
            double normalized_intensity = (pcl_cloud.points[i].intensity - min_intensity) / intensity_range;
            int color_value = static_cast<int>(255 * normalized_intensity);
            cv::circle(image_with_points, points2D[i], 3, cv::Scalar(0, color_value, 255 - color_value), -1);
        }
    }


    cv::Mat resized_image;
    double scale_factor = 0.8; // Adjust this factor to resize the image (1.0 = original size, 0.5 = half size)
    cv::resize(image_with_points, resized_image, cv::Size(), scale_factor, scale_factor);

    // Display the image
    cv::imshow("LiDAR Points Projected onto Image", resized_image);
    cv::waitKey(1);
}

void updateAndDisplayTransform(int, void* userdata) {
    // Retrieve the data needed for the projection
    auto data = static_cast<std::tuple<pcl::PointCloud<pcl::PointXYZI>, cv::Mat, cv::Mat, Eigen::Matrix4f, Eigen::Matrix4f*>*>(userdata);
    auto& pcl_cloud = std::get<0>(*data);
    auto& image = std::get<1>(*data);
    auto& intrinsic_matrix = std::get<2>(*data);
    auto& initial_transform = std::get<3>(*data);
    auto& updated_transform = *std::get<4>(*data);

    int max_position = 1800; // Increased range for more precision
    int initial_position = 900; // Middle position represents 0 degrees

    // Convert trackbar position to angle in radians, with 0 degrees at the center of the trackbar range
    double roll_rad = (cv::getTrackbarPos("Roll", "LiDAR Projection") - initial_position) * M_PI / 1800.0; // Adjusted for smaller increments
    double pitch_rad = (cv::getTrackbarPos("Pitch", "LiDAR Projection") - initial_position) * M_PI / 1800.0; // Adjusted for smaller increments
    double yaw_rad = (cv::getTrackbarPos("Yaw", "LiDAR Projection") - initial_position) * M_PI / 1800.0; // Adjusted for smaller increments

    // Convert Euler angles to a quaternion
    Eigen::Quaterniond q_update = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());

    // Extract the current rotation from the initial transform
    Eigen::Matrix3d initial_rotation = initial_transform.block<3, 3>(0, 0).cast<double>();
    Eigen::Quaterniond q_initial(initial_rotation);

    // Combine the initial rotation with the new rotation
    Eigen::Quaterniond q_combined = q_update * q_initial;

    // Apply the new rotation to the initial transformation matrix
    updated_transform.block<3, 3>(0, 0) = q_combined.toRotationMatrix().cast<float>();

    // Project the LiDAR points onto the image using the updated transform
    std::vector<ProjectedPoint> projected_points;
    projectLiDARPoints(pcl_cloud, image, intrinsic_matrix, updated_transform, projected_points);
}

// Function to adjust the transform interactively
std::vector<ProjectedPoint> adjustTransformInteractively(
    const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud,
    const cv::Mat& image,
    const cv::Mat& intrinsic_matrix,
    bool first_time_processing) {

    static Eigen::Matrix4f updated_transform;
    static bool is_transform_initialized = false;  // Flag to check if the transform has been initialized

    if (!is_transform_initialized) {
        // If this is the first call, initialize the transform
        updated_transform = getInitialCameraToBaseLinkTransform();
        is_transform_initialized = true;
    }

    if (first_time_processing) {
        // Pack the data into a tuple for easy access in the callback
        auto data = std::make_tuple(pcl_cloud, image, intrinsic_matrix, updated_transform, &updated_transform);

        // Create a window
        cv::namedWindow("LiDAR Projection", cv::WINDOW_AUTOSIZE);

        // Create trackbars to adjust roll, pitch, and yaw
        int max_position = 1800;  // Increased range for smaller increments
        int initial_position = 900;  // Centered position representing 0 degrees
        cv::createTrackbar("Roll", "LiDAR Projection", nullptr, max_position, updateAndDisplayTransform, &data);
        cv::createTrackbar("Pitch", "LiDAR Projection", nullptr, max_position, updateAndDisplayTransform, &data);
        cv::createTrackbar("Yaw", "LiDAR Projection", nullptr, max_position, updateAndDisplayTransform, &data);

        // Set the trackbars to the center position to represent 0 degrees initially
        cv::setTrackbarPos("Roll", "LiDAR Projection", initial_position);
        cv::setTrackbarPos("Pitch", "LiDAR Projection", initial_position);
        cv::setTrackbarPos("Yaw", "LiDAR Projection", initial_position);

        // Initialize with the current angles
        updateAndDisplayTransform(0, &data);

        // Wait for the user to press ESC to exit
        while (true) {
            int key = cv::waitKey(30);
            if (key == 27) { // 'ESC' key to break
                break;
            }
        }
    }

    // Project LiDAR points using the final or previously set transform
    std::vector<ProjectedPoint> projected_points;
    projectLiDARPoints(pcl_cloud, image, intrinsic_matrix, updated_transform, projected_points);

    if (first_time_processing) {
        for (auto& proj_point : projected_points) {
            proj_point.updated_transform = updated_transform;
        }
    }

    return projected_points;  // Return the ProjectedPoint vector
}
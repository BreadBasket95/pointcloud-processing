#ifndef VOXELIZATION_H
#define VOXELIZATION_H

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <array>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <bag_data_reader.h>
#include <ros/time.h>  // Include the correct header for ros::Time
#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "projected_point.h"

using Vector3d = Eigen::Vector3d;

struct VoxelData {
    int num_hits = 0;
    int num_misses = 0;
    double intensity_sum = 0.0;
    double intensity_squared_sum = 0.0;
    double reflectivity_sum = 0.0;
    double reflectivity_squared_sum = 0.0;
    std::vector<Eigen::VectorXd> points;
    double mean_intensity = 0.0;
    double variance_intensity = 0.0;
    double mean_reflectivity = 0.0;
    double variance_reflectivity = 0.0;
    double l_occ = 0.0;
    int N_occ = 0;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    double force = 0.0;
    Eigen::Vector3f color = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
};
using VoxelMap = std::map<std::array<int, 3>, VoxelData>;

class Voxelization {
public:
    Voxelization(double voxel_size, const Eigen::Vector3d& grid_min_bound, const Eigen::Vector3d& grid_max_bound);
    
    void processPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                           const std::vector<Eigen::Vector3d>& vehicle_positions,
                           const std::set<std::array<int, 3>>& relevant_voxels,
                           const std::vector<std::pair<double, Eigen::Vector3d>>& force_measurements,
                           const geometry_msgs::TransformStamped& latest_transform,
                           const cv::Mat& latest_image);
    
    void calculateVoxelFeatures();
    void saveVoxelMapToCSV(const std::string& filename, const std::set<std::array<int, 3>>& relevant_voxels);
    void fastVoxelTraversal(const std::vector<Eigen::Vector3d>& ray_directions, const Eigen::Vector3d& vehicle_position, const std::vector<double>& intensities, const std::vector<double>& reflectivities, const std::set<std::array<int, 3>>& relevant_voxels);
    void assignRGBToVoxels(
        const std::vector<ProjectedPoint>& projected_points,
        const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud,
        const cv::Mat& image, 
        const cv::Mat& intrinsic_matrix, 
        const Eigen::Matrix4f& camera_to_base_link, 
        const Eigen::Matrix4f& odom_to_base_link, 
        const Eigen::Vector3d& grid_min_bound, 
        double voxel_size);
    //void assignForcesToVoxels(const std::vector<std::pair<double, Eigen::Vector3d>>& force_measurements, const std::set<std::array<int, 3>>& relevant_voxels, double influence_radius);
    void assignForcesToVoxels(const std::set<std::array<int, 3>>& relevant_voxels, double influence_radius);
    std::set<std::array<int, 3>> computeVoxelIndicesWithinTrajectory(const std::vector<Eigen::Vector3d>& positions, double radius);
    void clearVoxelMap();
    std::map<std::array<int, 3>, double> force_voxel_map_;

    static std::map<ros::Time, cv::Mat> image_map;  // Static member for shared image data
    static ros::Time lidar_timestamp; 
    static std::map<ros::Time, geometry_msgs::TransformStamped> transform_map;  // Declare transform_map
    static std::array<int, 3> positionToVoxelCoords(const Eigen::Vector3d& position, const Eigen::Vector3d& grid_min_bound, double voxel_size);
    bool first_time_processing_ = true;  // Flag to check if it's the first time processing a point cloud
    Eigen::Matrix4f camera_to_base_link_;  // Store the last transform
    

private:

    double voxel_size;
    Eigen::Vector3d grid_min_bound;
    Eigen::Vector3d grid_max_bound;
    VoxelMap voxel_map;
    // Private utility functions
    cv::Point2d projectToCameraPlane(const Eigen::Vector3d& camera_point, const cv::Mat& intrinsic_matrix);
    cv::Vec3b getRGBValueAtPixel(const cv::Mat& image, const cv::Point2d& pixel);

    // Static functions and members are removed since only the latest image and transform are stored
    Eigen::Matrix4f getCameraToBaseLinkTransform() const;
    cv::Mat getCameraIntrinsicMatrix() const;
    std::vector<double> extractIntensities(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud) const;
    std::vector<double> extractReflectivities(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud) const;
};


#endif // VOXELIZATION_H
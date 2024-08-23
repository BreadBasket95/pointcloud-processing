#include "voxelization.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include "transform_adjuster.h"
#include <chrono>
#include "projected_point.h"

ros::Time Voxelization::lidar_timestamp;


Voxelization::Voxelization(double voxel_size, const Eigen::Vector3d& grid_min_bound, const Eigen::Vector3d& grid_max_bound)
    : voxel_size(voxel_size), grid_min_bound(grid_min_bound), grid_max_bound(grid_max_bound) {}

void Voxelization::processPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                                     const std::vector<Eigen::Vector3d>& vehicle_positions,
                                     const std::set<std::array<int, 3>>& relevant_voxels,
                                     const std::vector<std::pair<double, Eigen::Vector3d>>& force_measurements,
                                     const geometry_msgs::TransformStamped& latest_transform,
                                     const cv::Mat& latest_image) {

    /// temp timing
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud);

    // Get the transformation matrix from the latest transform
    Eigen::Matrix4f odom_to_base_link = BagDataReader::getTransformMatrix(latest_transform);
    std::vector<ProjectedPoint> projected_points = adjustTransformInteractively(pcl_cloud, latest_image, getCameraIntrinsicMatrix(), Voxelization::first_time_processing_);

    if (Voxelization::first_time_processing_) {
        Voxelization::camera_to_base_link_ = projected_points[0].updated_transform;
        Voxelization::first_time_processing_ = false;  // Set the flag to false after the first time
    }
    // Combined transformation from odom to camera
    Eigen::Matrix4f odom_to_camera = Voxelization::camera_to_base_link_ * odom_to_base_link;

    // Transform points to the odom and camera frames simultaneously
    std::vector<Eigen::Vector3d> points_in_odom;

    for (const auto& point : pcl_cloud.points) {
        Eigen::Vector4f point_vec(point.x, point.y, point.z, 1.0);
        // Transform to the odom frame
        Eigen::Vector4f point_odom = odom_to_base_link * point_vec;
        points_in_odom.emplace_back(point_odom.x(), point_odom.y(), point_odom.z());
        
    }
    
    std::vector<Eigen::Vector3d> ray_directions = points_in_odom;

    // Pass the transformed vehicle position as the origin point of the rays
    Eigen::Vector3d vehicle_position_in_odom(odom_to_base_link(0, 3), odom_to_base_link(1, 3), odom_to_base_link(2, 3)); 

    fastVoxelTraversal(ray_directions, vehicle_position_in_odom, extractIntensities(pcl_cloud), extractReflectivities(pcl_cloud), relevant_voxels);
    assignRGBToVoxels(projected_points, pcl_cloud, latest_image, getCameraIntrinsicMatrix(), Voxelization::camera_to_base_link_, odom_to_base_link, grid_min_bound, voxel_size);

    calculateVoxelFeatures();

    assignForcesToVoxels(relevant_voxels, 15);
}

Eigen::Matrix4f Voxelization::getCameraToBaseLinkTransform() const {
    // Define the transformation from camera to base_link (static for now)
    //Eigen::Vector3d translation(0.039145, 0.3005, -0.021825);
    Eigen::Vector3d translation(0.039145, 0.3005, -0.021825);
    //Eigen::Quaterniond quaternion(0.51582142, -0.46632017, 0.49136039, -0.52444136);
    Eigen::Quaterniond quaternion(0.50995227, -0.47273134, 0.49787735, -0.51825854);
    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation_matrix.cast<float>();
    transform.block<3, 1>(0, 3) = translation.cast<float>();

    return transform;
}

cv::Mat Voxelization::getCameraIntrinsicMatrix() const {
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1813.36, 0, 979.998,
                                                       0, 1810.89, 633.568,
                                                       0, 0, 1);
    return camera_matrix;
}

std::vector<double> Voxelization::extractIntensities(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud) const {
    std::vector<double> intensities;
    for (const auto& point : pcl_cloud.points) {
        intensities.push_back(point.intensity);
    }
    return intensities;
}

std::vector<double> Voxelization::extractReflectivities(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud) const {
    // If reflectivity data is stored in the intensity field, modify accordingly.
    std::vector<double> reflectivities;
    for (const auto& point : pcl_cloud.points) {
        reflectivities.push_back(point.intensity);  // Adjust if necessary
    }
    return reflectivities;
}

void Voxelization::calculateVoxelFeatures() {
    for (auto& kv : voxel_map) {
        VoxelData& voxel = kv.second;
        size_t point_count = voxel.points.size();
        if (point_count > 0) {
            Eigen::VectorXd intensities(point_count);
            Eigen::VectorXd reflectivities(point_count);
            for (size_t i = 0; i < point_count; ++i) {
                intensities[i] = voxel.points[i][3];
                reflectivities[i] = voxel.points[i][4];
            }
            voxel.mean_intensity = intensities.mean();
            voxel.variance_intensity = (intensities.array() - voxel.mean_intensity).square().mean();
            voxel.mean_reflectivity = reflectivities.mean();
            voxel.variance_reflectivity = (reflectivities.array() - voxel.mean_reflectivity).square().mean();

            if (point_count > 1) {
                Eigen::MatrixXd points_matrix(3, point_count);
                for (size_t i = 0; i < point_count; ++i) {
                    points_matrix.col(i) = voxel.points[i].head<3>();
                }
                Eigen::Vector3d centroid = points_matrix.rowwise().mean();
                Eigen::MatrixXd centered = points_matrix.colwise() - centroid;
                voxel.covariance = (centered * centered.transpose()) / double(point_count - 1);
            } else {
                voxel.covariance.setZero();
            }

            voxel.N_occ = voxel.num_hits;
            voxel.l_occ = std::log(double(voxel.num_hits) / (voxel.num_misses + 1e-6));
        }
    }
}
void Voxelization::assignForcesToVoxels(const std::set<std::array<int, 3>>& relevant_voxels, double influence_radius) {
    int countApplied = 0;
    int countNotFound = 0;

    for (const auto& voxel : relevant_voxels) {
        bool forceApplied = false;

        for (const auto& force_voxel_pair : force_voxel_map_) {
            const auto& force_voxel = force_voxel_pair.first;
            double force = force_voxel_pair.second;

            // Calculate the distance between the current voxel and the force voxel
            double distance = std::hypot(voxel[0] - force_voxel[0], voxel[1] - force_voxel[1]);

            // Check if the voxel is within the influence radius
            if (distance <= influence_radius) {
                auto& voxel_data = voxel_map[voxel];

                // Apply the force only if it's greater than the existing force
                if (force > voxel_data.force) {
                    voxel_data.force = force;
                    forceApplied = true;
                }
            }
        }

        if (forceApplied) {
            countApplied++;
        } else {
            countNotFound++;
        }
    }

}

void Voxelization::fastVoxelTraversal(const std::vector<Eigen::Vector3d>& ray_directions, const Eigen::Vector3d& vehicle_position, const std::vector<double>& intensities, const std::vector<double>& reflectivities, const std::set<std::array<int, 3>>& relevant_voxels) {
// Ensure that there are ray directions to process
        assert(!ray_directions.empty());

    // Initialize matrices to store the starting and ending voxel coordinates for each ray
    Eigen::Matrix<int, Eigen::Dynamic, 3> start_voxels(ray_directions.size(), 3);
    Eigen::Matrix<int, Eigen::Dynamic, 3> end_voxels(ray_directions.size(), 3);

    // Loop through each ray direction to calculate the start and end voxel coordinates
    for (size_t i = 0; i < ray_directions.size(); ++i) {
        // Convert the vehicle position to voxel coordinates
        std::array<int, 3> start_voxel_coords = Voxelization::positionToVoxelCoords(vehicle_position, grid_min_bound, voxel_size);
        start_voxels.row(i) << start_voxel_coords[0], start_voxel_coords[1], start_voxel_coords[2];

        // Calculate the endpoint of the ray
        Eigen::Vector3d end_point = ray_directions[i];
        // Convert the end point to voxel coordinates
        std::array<int, 3> end_voxel_coords = Voxelization::positionToVoxelCoords(end_point, grid_min_bound, voxel_size);
// Store the end voxel coordinates
                end_voxels.row(i) << end_voxel_coords[0], end_voxel_coords[1], end_voxel_coords[2];
    }

    // Initialize a vector to keep track of which rays are still active (i.e., haven't reached their end voxel)
    Eigen::VectorXi active_rays = Eigen::VectorXi::Ones(ray_directions.size());
    
    size_t iteration = 0;
    // Loop until all rays have reached their end voxels
    while (active_rays.sum() > 0) {
        // Calculate the remaining distance for each ray to travel in voxel coordinates
        Eigen::Matrix<int, Eigen::Dynamic, 3> remaining_distance = end_voxels - start_voxels;
// Determine the voxel component (x, y, or z) with the maximum distance to travel for each ray
                Eigen::VectorXi max_distance_component = remaining_distance.cwiseAbs().rowwise().maxCoeff();

        // Loop through each ray
        for (int i = 0; i < ray_directions.size(); ++i) {
            if (active_rays[i]) {
                // Calculate the absolute remaining distance in each component
                Eigen::Vector3i abs_remaining_distance = remaining_distance.row(i).cwiseAbs();
// Determine the component (x, y, or z) with the maximum remaining distance
                                int component = 0;
                if (abs_remaining_distance[1] > abs_remaining_distance[0]) {
                    component = 1;
                }
                if (abs_remaining_distance[2] > abs_remaining_distance[component]) {
                    component = 2;
                }
                // Determine the direction of movement in the selected component
                int direction = (remaining_distance(i, component) > 0) ? 1 : -1;
// Update the start voxel coordinate in the selected component
                start_voxels(i, component) += direction;
            }
        }

        // Determine which rays have reached their end voxels in this iteration
        Eigen::VectorXi reached_end_now = (start_voxels.array() == end_voxels.array()).rowwise().all().cast<int>();
        // Loop through each ray to process those that have reached their end voxels
        for (int i = 0; i < ray_directions.size(); ++i) {
                if (reached_end_now[i] && active_rays[i]) {
                // Get the current voxel coordinates for the ray
                auto voxel_coords = std::array<int, 3>{start_voxels(i, 0), start_voxels(i, 1), start_voxels(i, 2)};
// Check if the voxel is relevant (i.e., within the area of interest)
                                if (relevant_voxels.find(voxel_coords) != relevant_voxels.end()) {
                    // Update voxel data for hits and sums
                    voxel_map[voxel_coords].num_hits++;
                    voxel_map[voxel_coords].intensity_sum += intensities[i];
                    voxel_map[voxel_coords].intensity_squared_sum += intensities[i] * intensities[i];
                    voxel_map[voxel_coords].reflectivity_sum += reflectivities[i];
                    voxel_map[voxel_coords].reflectivity_squared_sum += reflectivities[i] * reflectivities[i];
                    // Store the point data in the voxel
                    Eigen::VectorXd point_data(5);
                    point_data << ray_directions[i][0], ray_directions[i][1], ray_directions[i][2], intensities[i], reflectivities[i];
                    voxel_map[voxel_coords].points.push_back(point_data);
                }
                // Mark the ray as inactive
                active_rays[i] = 0;
}
        }

        // Loop through each ray to update voxel data for misses
        for (int i = 0; i < ray_directions.size(); ++i) {
                if (active_rays[i]) {
                // Get the current voxel coordinates for the ray
                auto voxel_coords = std::array<int, 3>{start_voxels(i, 0), start_voxels(i, 1), start_voxels(i, 2)};
// Check if the voxel is relevant (i.e., within the area of interest)
                                if (relevant_voxels.find(voxel_coords) != relevant_voxels.end()) {
                    // Update voxel data for misses
                    voxel_map[voxel_coords].num_misses++;
                }
            }
        }
    }
}

void Voxelization::assignRGBToVoxels(
    const std::vector<ProjectedPoint>& projected_points,
    const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud,
    const cv::Mat& image, 
    const cv::Mat& intrinsic_matrix, 
    const Eigen::Matrix4f& camera_to_base_link, 
    const Eigen::Matrix4f& odom_to_base_link, 
    const Eigen::Vector3d& grid_min_bound, 
    double voxel_size)
    {
    if (image.empty()) {
        std::cerr << "Error: The provided image is empty. Cannot assign colors or display projected LiDAR points." << std::endl;
        return;
    }
    cv::Mat image_with_points = image.clone();
    std::vector<Eigen::Vector3d> points_in_odom;

    for (const auto& point : pcl_cloud.points) {
        Eigen::Vector4f point_vec(point.x, point.y, point.z, 1.0);
        // Transform to the odom frame
        Eigen::Vector4f point_odom = odom_to_base_link * point_vec;
        points_in_odom.emplace_back(point_odom.x(), point_odom.y(), point_odom.z());
        
    }
    for (size_t i = 0; i < projected_points.size(); ++i) {

        const auto& proj_point = projected_points[i];
        if (proj_point.point3D.z() <= 0) {
            continue;
        }

        if (proj_point.point2D.x < 0 || proj_point.point2D.y < 0 || 
            proj_point.point2D.x >= image.cols || proj_point.point2D.y >= image.rows) {
            continue;
        }

        // Get the RGB color from the image
        cv::Vec3b rgb = image.at<cv::Vec3b>(cv::Point2d(proj_point.point2D));
        Eigen::Vector3f voxel_rgb(rgb[2] / 255.0f, rgb[1] / 255.0f, rgb[0] / 255.0f);

        std::array<int, 3> voxel_coords = Voxelization::positionToVoxelCoords(points_in_odom[i], grid_min_bound, voxel_size);
        
        // Accumulate the RGB values and count the number of points in the voxel
        if (voxel_map.find(voxel_coords) != voxel_map.end()) {
            VoxelData new_voxel;
            VoxelData& voxel = voxel_map[voxel_coords];
            voxel.color += voxel_rgb;
        }else {
            VoxelData new_voxel;
            new_voxel.color = voxel_rgb;
            voxel_map[voxel_coords] = new_voxel;
        }
    }

    // Average the RGB values for each voxel after accumulation
    for (auto& kv : voxel_map) {
        VoxelData& voxel = kv.second;
        if (voxel.num_hits >= 1) {
            voxel.color /= voxel.num_hits;
        }
    }
    
}

std::array<int, 3> Voxelization::positionToVoxelCoords(const Eigen::Vector3d& position, const Eigen::Vector3d& grid_min_bound, double voxel_size) {
    return {(int)std::floor((position[0] - grid_min_bound[0]) / voxel_size),
            (int)std::floor((position[1] - grid_min_bound[1]) / voxel_size),
            (int)std::floor((position[2] - grid_min_bound[2]) / voxel_size)};
}

void Voxelization::clearVoxelMap() {
    voxel_map.clear();
}

void Voxelization::saveVoxelMapToCSV(const std::string& filename, const std::set<std::array<int, 3>>& relevant_voxels) {
    std::ofstream csvfile(filename);
    csvfile << "VoxelX,VoxelY,VoxelZ,NumHits,NumMisses,MeanIntensity,VarianceIntensity,MeanReflectivity,VarianceReflectivity,Force,N_occ,l_occ,R,G,B,Covariance\n";
    
    for (const auto& kv : voxel_map) {
        const auto& voxel_coords = kv.first;
        const auto& voxel = kv.second;

        // Only save voxels that are within the relevant_voxels set
        if (relevant_voxels.find(voxel_coords) != relevant_voxels.end()) {

            if (voxel.num_hits >= 1 || voxel.num_misses >= 1) {
                csvfile << voxel_coords[0] << "," << voxel_coords[1] << "," << voxel_coords[2] << ","
                        << voxel.num_hits << "," << voxel.num_misses << ","
                        << voxel.mean_intensity << "," << voxel.variance_intensity << ","
                        << voxel.mean_reflectivity << "," << voxel.variance_reflectivity << ","
                        << voxel.force << "," << voxel.N_occ << "," << voxel.l_occ;

                // Only add the RGB values if there is a hit
                        csvfile << "," << voxel.color[0] << "," << voxel.color[1] << "," << voxel.color[2];
                    // If no hit, leave the RGB columns empty

                // Write the covariance matrix values
                csvfile << ",";
                for (int i = 0; i < voxel.covariance.size(); ++i) {
                    csvfile << voxel.covariance(i);
                    if (i < voxel.covariance.size() - 1) {
                        csvfile << ",";
                    }
                }
                csvfile << "\n";
            
                if (!csvfile) {
                    std::cerr << "Error writing to CSV file: " << filename << std::endl;
                }
            }

        }
    }
    std::cout << "Voxel map saved to " << filename << std::endl;
    csvfile.close();
}


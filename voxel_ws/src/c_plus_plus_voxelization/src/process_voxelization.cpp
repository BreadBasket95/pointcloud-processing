#include "process_voxelization.h"
#include "bag_data_reader.h"
#include "voxelization.h"
#include <Eigen/Dense>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <filesystem>  

void ProcessVoxelization::processVoxelization(const std::string& bag_file, const std::string& point_cloud_topic,
                                              const std::string& tf_topic, const std::string& loadcell_one_topic,
                                              const std::string& loadcell_two_topic, const std::string& camera_topic,
                                              double voxel_size) {
    // Preprocessed data from bag_data_reader
    std::vector<Eigen::Vector3d> vehicle_positions;
    std::vector<std::pair<double, Eigen::Vector3d>> force_measurements;
    Eigen::Vector3d min_bound, max_bound;
    std::set<std::array<int, 3>> relevant_voxels;
    std::map<std::array<int, 2>, double> force_voxel_map;

    // Call the bag_data_reader to extract vehicle trajectory, min/max bounds, and force measurements
    BagDataReader data_reader;
    data_reader.readDataFromBag(bag_file, point_cloud_topic, tf_topic, loadcell_one_topic, loadcell_two_topic,
                                vehicle_positions, force_measurements, min_bound, max_bound, voxel_size, relevant_voxels);

    if (vehicle_positions.empty()) {
        std::cerr << "No vehicle positions found in the bag file." << std::endl;
        return;
    }

    Eigen::Vector3d grid_min_bound = min_bound - Eigen::Vector3d::Ones() * voxel_size;
    Eigen::Vector3d grid_max_bound = max_bound + Eigen::Vector3d::Ones() * voxel_size;

    Voxelization voxelizer(voxel_size, grid_min_bound, grid_max_bound);
    voxelizer.first_time_processing_ = true;
    voxelizer.force_voxel_map_.clear();

    for (const auto& force_measurement : force_measurements) {
        const auto& force_position = force_measurement.second;
        double force = force_measurement.first;

        Eigen::Vector2d force_position_xy = force_position.head<2>();
        std::array<int, 2> force_voxel_xy = {
            static_cast<int>(std::floor((force_position_xy[0] - grid_min_bound[0]) / voxel_size)),
            static_cast<int>(std::floor((force_position_xy[1] - grid_min_bound[1]) / voxel_size))
        };

        // Populate the force voxel map
        std::array<int, 3> force_voxel = {force_voxel_xy[0], force_voxel_xy[1], static_cast<int>(std::floor((force_position[2] - grid_min_bound[2]) / voxel_size))};

        if (voxelizer.force_voxel_map_.find(force_voxel) == voxelizer.force_voxel_map_.end() || force > voxelizer.force_voxel_map_[force_voxel]) {
            voxelizer.force_voxel_map_[force_voxel] = force;
        }
    }

    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (rosbag::BagIOException& e) {
        throw;
    }

    std::vector<std::string> topics = {point_cloud_topic, tf_topic, camera_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    geometry_msgs::TransformStamped latest_transform;
    cv::Mat latest_image;

    size_t cloud_index = 0;

    // Extract the filename from the bag file path
    std::string bag_filename = std::filesystem::path(bag_file).filename().string();
    
    // Remove the .bag extension
    std::string bag_name = bag_filename.substr(0, bag_filename.find_last_of('.'));
    
    // Create the output directory path
    std::string output_dir = "/mnt/6C20E58F769B49C6/Users/Marc/Desktop/all_veg_override_rosbag_grp_4/" + bag_name;

    // Create the directory if it doesn't exist
    std::filesystem::create_directories(output_dir);

    int pointcloud_count = 0;
    for (const rosbag::MessageInstance& m : view) {
        if (m.getTopic() == point_cloud_topic) {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg) {
                // Process the point cloud with the latest transform and image
                if (pointcloud_count % 50 == 0) {
                    std::cout << "Reset lidar-camera transform" << std::endl;
                    voxelizer.first_time_processing_ = true;
                }
                voxelizer.processPointCloud(cloud_msg, vehicle_positions, relevant_voxels, force_measurements, latest_transform, latest_image);

                // Save the voxel map for the current point cloud
                std::string output_csv = output_dir + "/voxel_map_" + std::to_string(cloud_index++) + ".csv";
                voxelizer.saveVoxelMapToCSV(output_csv, relevant_voxels);
                voxelizer.clearVoxelMap();
                pointcloud_count++;
            }
        } else if (m.getTopic() == camera_topic) {
            sensor_msgs::CompressedImage::ConstPtr image_msg = m.instantiate<sensor_msgs::CompressedImage>();
            if (image_msg) {
                // Store the latest camera image
                cv::Mat raw_data(1, image_msg->data.size(), CV_8UC1, (void*)(&image_msg->data[0]));
                latest_image = cv::imdecode(raw_data, cv::IMREAD_COLOR);
            }
        } else if (m.getTopic() == tf_topic) {
            tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
            if (tf_msg) {
                // Store the latest transform
                for (const auto& transform : tf_msg->transforms) {
                    if (transform.header.frame_id == "odom" && transform.child_frame_id == "base_link") {
                        latest_transform = transform;
                    }
                }
            }
        }
    }
    bag.close();

    std::cout << "Voxelization complete. Voxel maps saved for each processed point cloud." << std::endl;
}

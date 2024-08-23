#include "bag_data_reader.h"
#include "voxelization.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "c_plus_plus_voxelization/weight.h"
#include <map>
#include <ros/time.h>
#include <Eigen/Dense>


// Function to get the transformation matrix from a TransformStamped message
Eigen::Matrix4f BagDataReader::getTransformMatrix(const geometry_msgs::TransformStamped& transform) {
    Eigen::Affine3d affine_transform = Eigen::Affine3d::Identity();
    affine_transform.translation() << transform.transform.translation.x,
                                      transform.transform.translation.y,
                                      transform.transform.translation.z;
    Eigen::Quaterniond q(transform.transform.rotation.w,
                         transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z);
    affine_transform.rotate(q);
    return affine_transform.matrix().cast<float>();  // Ensure this returns a 4x4 matrix
}

// Function to apply the sensor offset to a transform
Eigen::Vector3d BagDataReader::applySensorOffset(const geometry_msgs::TransformStamped& transform, const Eigen::Vector3d& offset) {
    Eigen::Matrix4f transform_matrix = getTransformMatrix(transform);
    Eigen::Vector4f offset_homogeneous(offset.x(), offset.y(), offset.z(), 1.0);
    Eigen::Vector4f transformed_offset = transform_matrix * offset_homogeneous;


    return Eigen::Vector3d(transformed_offset.x(), transformed_offset.y(), transformed_offset.z());
}

// Function to read data from the ROS bag file and preprocess the necessary information
void BagDataReader::readDataFromBag(const std::string& bag_file, const std::string& point_cloud_topic,
                                    const std::string& tf_topic, const std::string& loadcell_one_topic, 
                                    const std::string& loadcell_two_topic, std::vector<Eigen::Vector3d>& vehicle_positions, 
                                    std::vector<std::pair<double, Eigen::Vector3d>>& force_measurements,
                                    Eigen::Vector3d& min_bound, Eigen::Vector3d& max_bound,
                                    double voxel_size, std::set<std::array<int, 3>>& relevant_voxels) {

    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (rosbag::BagIOException& e) {
        throw;
    }

    std::vector<std::string> topics = {point_cloud_topic, tf_topic, loadcell_one_topic, loadcell_two_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    geometry_msgs::TransformStamped last_transform;
    bool last_transform_received = false;
    float last_loadcell_one_val = 0.0;
    float last_loadcell_two_val = 0.0;
    bool loadcell_one_received = false;
    bool loadcell_two_received = false;

    Eigen::Vector3d sensor_offset(0.4, 0.0, 0.0);
    
    std::vector<std::vector<Eigen::Vector3d>> all_points;  // Store all points for min/max calculation later

    for (const rosbag::MessageInstance& m : view) {
        if (m.getTopic() == point_cloud_topic) {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg == nullptr || !last_transform_received) {
                continue;
            }
            
            // Transform the point cloud using the latest transform
            Eigen::Matrix4f transform_matrix = getTransformMatrix(last_transform);
            pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
            pcl::fromROSMsg(*cloud_msg, pcl_cloud);

            std::vector<Eigen::Vector3d> points;

            for (auto& point : pcl_cloud.points) {
                Eigen::Vector4f point_vec(point.x, point.y, point.z, 1.0);
                Eigen::Vector4f transformed_point = transform_matrix * point_vec;
                Eigen::Vector3d point_xyz(transformed_point.x(), transformed_point.y(), transformed_point.z());
                points.push_back(point_xyz);
            }

            all_points.push_back(points);

            // Add the vehicle position based on the latest transform
            vehicle_positions.emplace_back(
                last_transform.transform.translation.x,
                last_transform.transform.translation.y,
                last_transform.transform.translation.z
            );

        } else if (m.getTopic() == tf_topic) {
            tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
            if (tf_msg != nullptr) {
                for (const geometry_msgs::TransformStamped& transform : tf_msg->transforms) {
                    if (transform.header.frame_id == "odom" && transform.child_frame_id == "base_link") {
                        last_transform = transform;
                        last_transform_received = true;
                    }
                }
            }
        } else if (m.getTopic() == loadcell_one_topic) {
            //std::cout << "Processing loadcell one message." << std::endl;
            c_plus_plus_voxelization::weight::ConstPtr loadcell_msg = m.instantiate<c_plus_plus_voxelization::weight>();
            if (loadcell_msg != nullptr) {
                last_loadcell_one_val = loadcell_msg->lbs;
                loadcell_one_received = true;

                if (loadcell_two_received && last_transform_received) {
                    double combined_force = last_loadcell_one_val + last_loadcell_two_val;
                    Eigen::Vector3d force_position = applySensorOffset(last_transform, sensor_offset);
                    
                    force_measurements.emplace_back(combined_force, force_position);
                }
            }
            else {
                std::cerr << "Error: loadcell_msg is nullptr." << std::endl;
            }
        } else if (m.getTopic() == loadcell_two_topic) {
            //std::cout << "Processing loadcell two message." << std::endl;
            c_plus_plus_voxelization::weight::ConstPtr loadcell_msg = m.instantiate<c_plus_plus_voxelization::weight>();
            if (loadcell_msg != nullptr) {
                last_loadcell_two_val = loadcell_msg->lbs;
                loadcell_two_received = true;

                if (loadcell_one_received && last_transform_received) {
                    double combined_force = last_loadcell_one_val + last_loadcell_two_val;
                    Eigen::Vector3d force_position = applySensorOffset(last_transform, sensor_offset);
                    force_measurements.emplace_back(combined_force, force_position);

                }
            }
            else {
                std::cerr << "Error: loadcell_msg is nullptr." << std::endl;
            }
        }
    }
    bag.close();

    if (vehicle_positions.empty() || all_points.empty()) {
        std::cerr << "No vehicle positions or points found in the bag file." << std::endl;
        return;
    }

    // Initialize the min and max bounds using the first vehicle position
    min_bound = vehicle_positions.front();
    max_bound = vehicle_positions.front();

    // Update bounds based on vehicle positions
    for (const auto& pos : vehicle_positions) {
        for (int i = 0; i < 3; ++i) {
            if (pos[i] < min_bound[i]) min_bound[i] = pos[i];
            if (pos[i] > max_bound[i]) max_bound[i] = pos[i];
        }
    }

    // Update bounds based on point cloud data
    for (const auto& points : all_points) {
        for (const auto& point : points) {
            for (int i = 0; i < 3; ++i) {
                if (point[i] < min_bound[i]) min_bound[i] = point[i];
                if (point[i] > max_bound[i]) max_bound[i] = point[i];
            }
        }
    }

    // Expand bounds slightly to ensure all points fit within the grid
    Eigen::Vector3d grid_min_bound = min_bound - Eigen::Vector3d::Ones() * voxel_size;
    Eigen::Vector3d grid_max_bound = max_bound + Eigen::Vector3d::Ones() * voxel_size;

    // Calculate relevant voxel indices within the trajectory
    Voxelization voxelizer(voxel_size, grid_min_bound, grid_max_bound);
    relevant_voxels = BagDataReader::computeVoxelIndicesWithinTrajectory(vehicle_positions, voxel_size, grid_min_bound, 1.5);
    // Output the calculated bounds for verification
    std::cout << "Calculated grid_min_bound: {" << grid_min_bound[0] << ", " << grid_min_bound[1] << ", " << grid_min_bound[2] << "}" << std::endl;
    std::cout << "Calculated grid_max_bound: {" << grid_max_bound[0] << ", " << grid_max_bound[1] << ", " << grid_max_bound[2] << "}" << std::endl;
}

// Function to compute relevant voxel indices within the vehicle trajectory
std::set<std::array<int, 3>> BagDataReader::computeVoxelIndicesWithinTrajectory(const std::vector<Eigen::Vector3d>& positions, double voxel_size, const Eigen::Vector3d& grid_min_bound, double radius) {
    std::set<std::array<int, 3>> relevant_voxels;
    int voxel_radius = static_cast<int>(std::ceil(radius / voxel_size));

    for (const auto& pos : positions) {
        auto voxel_index = Voxelization::positionToVoxelCoords(pos, grid_min_bound, voxel_size);
        Eigen::Vector3d voxel_center = grid_min_bound + voxel_size * Eigen::Vector3d(voxel_index[0], voxel_index[1], voxel_index[2]);

        for (int dx = -voxel_radius; dx <= voxel_radius; ++dx) {
            for (int dy = -voxel_radius; dy <= voxel_radius; ++dy) {
                for (int dz = -voxel_radius; dz <= voxel_radius; ++dz) {
                    Eigen::Vector3d offset_voxel = voxel_center + voxel_size * Eigen::Vector3d(dx, dy, dz);
                    if ((offset_voxel.head<2>() - pos.head<2>()).norm() <= radius) {
                        std::array<int, 3> new_voxel_index = {voxel_index[0] + dx, voxel_index[1] + dy, voxel_index[2] + dz};
                        relevant_voxels.insert(new_voxel_index);
                    }
                }
            }
        }
    }

    return relevant_voxels;
}


// #include "bag_data_reader.h"
// #include "voxelization.h"
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <tf2_msgs/TFMessage.h>
// #include <cv_bridge/cv_bridge.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include "c_plus_plus_voxelization/weight.h"
// #include <sensor_msgs/CompressedImage.h>
// #include <map>
// #include <ros/time.h>


// Eigen::Matrix4f BagDataReader::getTransformMatrix(const geometry_msgs::TransformStamped& transform) {
//     Eigen::Affine3d affine_transform = Eigen::Affine3d::Identity();
//     affine_transform.translation() << transform.transform.translation.x,
//                                       transform.transform.translation.y,
//                                       transform.transform.translation.z;
//     Eigen::Quaterniond q(transform.transform.rotation.w,
//                          transform.transform.rotation.x,
//                          transform.transform.rotation.y,
//                          transform.transform.rotation.z);
//     affine_transform.rotate(q);
//     return affine_transform.matrix().cast<float>();  // Ensure this returns a 4x4 matrix
// }

// Eigen::Vector3d BagDataReader::applySensorOffset(const geometry_msgs::TransformStamped& transform, const Eigen::Vector3d& offset) {
//     Eigen::Matrix4f transform_matrix = getTransformMatrix(transform);
//     Eigen::Vector4f offset_homogeneous(offset.x(), offset.y(), offset.z(), 1.0);
//     Eigen::Vector4f transformed_offset = transform_matrix * offset_homogeneous;
//     return Eigen::Vector3d(transformed_offset.x(), transformed_offset.y(), transformed_offset.z());
// }

// void BagDataReader::readDataFromBag(const std::string& bag_file, const std::string& point_cloud_topic,
//                                     const std::string& tf_topic, const std::string& loadcell_one_topic, 
//                                     const std::string& loadcell_two_topic, const std::string& camera_topic,
//                                     std::vector<std::vector<Eigen::Vector3d>>& all_points, 
//                                     std::vector<std::vector<double>>& all_intensities, 
//                                     std::vector<std::vector<double>>& all_reflectivities, 
//                                     std::vector<Eigen::Vector3d>& vehicle_positions, 
//                                     std::vector<std::pair<double, Eigen::Vector3d>>& force_measurements,
//                                     cv::Mat& last_image, cv::Mat& camera_matrix, cv::Mat& dist_coeffs,
//                                     Eigen::Matrix4f& lidar_to_camera_transform) {
//     rosbag::Bag bag;
//     try {
//         bag.open(bag_file, rosbag::bagmode::Read);
//     } catch (rosbag::BagIOException& e) {
//         throw;
//     }

//     std::vector<std::string> topics = {point_cloud_topic, camera_topic, tf_topic, loadcell_one_topic, loadcell_two_topic};
//     rosbag::View view(bag, rosbag::TopicQuery(topics));

//     geometry_msgs::TransformStamped last_transform;
//     bool last_transform_received = false;
//     float last_loadcell_one_val = 0.0;
//     float last_loadcell_two_val = 0.0;
//     bool loadcell_one_received = false;
//     bool loadcell_two_received = false;

//     Eigen::Vector3d sensor_offset(0.4, 0.0, 0.0);
    
//     //std::cout << "camera_topic: " << camera_topic << std::endl;

//     for (const rosbag::MessageInstance& m : view) {
//         if (m.getTopic() == point_cloud_topic) {
//             std::cout << "Processing point cloud message. Timestamp: " << m.getTime() << std::endl;
//             sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
//             if (cloud_msg == nullptr) {
//                 std::cerr << "Error: cloud_msg is nullptr." << std::endl;
//             } else if (!last_transform_received) {
//                 std::cerr << "Warning: Transform not received before point cloud processing." << std::endl;
//             } else {
//                 Voxelization::lidar_timestamp = cloud_msg->header.stamp;

//                 // Find the closest transformation in time
//                 auto closest_transform_it = Voxelization::transform_map.lower_bound(cloud_msg->header.stamp);
//                 if (closest_transform_it != Voxelization::transform_map.end()) {
//                     last_transform = closest_transform_it->second;

//                     // Add the closest transform to the transform_map with the LiDAR timestamp
//                     Voxelization::transform_map[Voxelization::lidar_timestamp] = last_transform;

//                     //std::cout << "Transform matrix: " << getTransformMatrix(last_transform)
//                     ///          << " at timestamp: " << last_transform.header.stamp << std::endl;
//                     //std::cout << "Adding transform to map with LiDAR timestamp: " << Voxelization::lidar_timestamp << std::endl;

//             } else {
//                 std::cerr << "No transform found for timestamp " << cloud_msg->header.stamp << std::endl;
//                 continue;
//             }

//             // Store the vehicle position using the closest transform
//             vehicle_positions.emplace_back(
//                 last_transform.transform.translation.x,
//                 last_transform.transform.translation.y,
//                 last_transform.transform.translation.z
//             );

//             // Process the point cloud with the correct transform
//             pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
//             pcl::fromROSMsg(*cloud_msg, pcl_cloud);
//             Eigen::Matrix4f transform_matrix = getTransformMatrix(last_transform);

//             std::vector<Eigen::Vector3d> points;
//             std::vector<double> intensities;
//             std::vector<double> reflectivities;

//             for (auto& point : pcl_cloud.points) {
//                 Eigen::Vector4f point_vec(point.x, point.y, point.z, 1.0);
//                 Eigen::Vector4f transformed_point = transform_matrix * point_vec;
//                 points.emplace_back(transformed_point.x(), transformed_point.y(), transformed_point.z());
//                 intensities.push_back(point.intensity);
//                 reflectivities.push_back(point.intensity);  // Adjust as needed
//             }

//             all_points.push_back(points);
//             all_intensities.push_back(intensities);
//             all_reflectivities.push_back(reflectivities);
//         }
//         } else if (m.getTopic() == tf_topic) {
//             tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
//             if (tf_msg != nullptr) {
//                 for (const geometry_msgs::TransformStamped& transform : tf_msg->transforms) {
//                     if (transform.header.frame_id == "odom" && transform.child_frame_id == "base_link") {
//                         // Store the transformation for later retrieval
//                         Voxelization::transform_map[transform.header.stamp] = transform;
//                         last_transform_received = true;
//                         Eigen::Matrix4f transform_matrix = getTransformMatrix(transform);
//                         //std::cout << "Storing Transform at Timestamp: " << transform.header.stamp
//                         //        << "\nTransform Matrix:\n" << transform_matrix << std::endl;
//                     }
//                 }
//             }
//         } else if (m.getTopic() == loadcell_one_topic) {
//             c_plus_plus_voxelization::weight::ConstPtr loadcell_msg = m.instantiate<c_plus_plus_voxelization::weight>();
//             if (loadcell_msg != nullptr) {
//                 last_loadcell_one_val = loadcell_msg->lbs;
//                 loadcell_one_received = true;

//                 if (loadcell_two_received && last_transform_received) {
//                     double combined_force = last_loadcell_one_val + last_loadcell_two_val;
//                     Eigen::Vector3d force_position = applySensorOffset(last_transform, sensor_offset);

//                     force_measurements.emplace_back(combined_force, force_position);
//                 }
//             }
//         } else if (m.getTopic() == loadcell_two_topic) {
//             c_plus_plus_voxelization::weight::ConstPtr loadcell_msg = m.instantiate<c_plus_plus_voxelization::weight>();
//             if (loadcell_msg != nullptr) {
//                 last_loadcell_two_val = loadcell_msg->lbs;
//                 loadcell_two_received = true;
//                 if (loadcell_one_received && last_transform_received) {
//                     double combined_force = last_loadcell_one_val + last_loadcell_two_val;
//                     Eigen::Vector3d force_position = applySensorOffset(last_transform, sensor_offset);

//                     force_measurements.emplace_back(combined_force, force_position);
//                 }
//             }
//         } else if (m.getTopic() == camera_topic) {  // Process the image topic
//             sensor_msgs::CompressedImage::ConstPtr compressed_image_msg = m.instantiate<sensor_msgs::CompressedImage>();
//             if (compressed_image_msg != nullptr) {
//                 //std::cout << "Compressed image message received. Format: " << compressed_image_msg->format << std::endl;
//                 try {
//                     // Decode the compressed image
//                     cv::Mat raw_data(1, compressed_image_msg->data.size(), CV_8UC1, (void*)(&compressed_image_msg->data[0]));
//                     cv::Mat decoded_image;
//                     if (compressed_image_msg->format.find("jpeg") != std::string::npos || compressed_image_msg->format.find("jpg") != std::string::npos) {
//                         decoded_image = cv::imdecode(raw_data, cv::IMREAD_COLOR);
//                     } else if (compressed_image_msg->format.find("png") != std::string::npos) {
//                         decoded_image = cv::imdecode(raw_data, cv::IMREAD_UNCHANGED);
//                     } else {
//                         std::cerr << "Unsupported compressed image format: " << compressed_image_msg->format << std::endl;
//                         return;
//                     }

//                     if (decoded_image.empty()) {
//                         std::cerr << "Warning: Decoded image is empty." << std::endl;
//                     } else {
//                         // Store the image along with its timestamp
//                         Voxelization::image_map[compressed_image_msg->header.stamp] = decoded_image;
//                         std::cout << "Image successfully decoded and stored. Timestamp: " 
//                                 << compressed_image_msg->header.stamp << std::endl;
//                     }
//                 } catch (cv::Exception& e) {
//                     std::cerr << "OpenCV exception: " << e.what() << std::endl;
//                 }
//             } else {
//                 std::cerr << "Error: compressed_image_msg is nullptr." << std::endl;
//             }
//         }
//     }
//     bag.close();
// }
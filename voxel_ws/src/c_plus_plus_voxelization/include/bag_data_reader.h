#ifndef BAG_DATA_READER_H
#define BAG_DATA_READER_H

#include "voxelization.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

class BagDataReader {
public:
    static Eigen::Matrix4f getTransformMatrix(const geometry_msgs::TransformStamped& transform);

    void readDataFromBag(const std::string& bag_file, const std::string& point_cloud_topic,
                         const std::string& tf_topic, const std::string& loadcell_one_topic, 
                         const std::string& loadcell_two_topic, 
                         std::vector<Eigen::Vector3d>& vehicle_positions, 
                         std::vector<std::pair<double, Eigen::Vector3d>>& force_measurements, 
                         Eigen::Vector3d& min_bound, Eigen::Vector3d& max_bound, 
                         double voxel_size, std::set<std::array<int, 3>>& relevant_voxels);
    std::set<std::array<int, 3>> computeVoxelIndicesWithinTrajectory(const std::vector<Eigen::Vector3d>& positions, double voxel_size, const Eigen::Vector3d& grid_min_bound, double radius);
private:
    Eigen::Vector3d applySensorOffset(const geometry_msgs::TransformStamped& transform, const Eigen::Vector3d& offset);
};

#endif // BAG_DATA_READER_H
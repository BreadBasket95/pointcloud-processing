// #include "voxelization.h"
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
// #include <sensor_msgs/PointCloud2.h>

// int main(int argc, char** argv) {
//     if (argc != 6) {
//         std::cerr << "Usage: " << argv[0] << " <data_directory> <bag_file> <point_cloud_topic> <tf_topic> <loadcell_topic>" << std::endl;
//         return 1;
//     }

//     std::string data_directory = argv[1];
//     std::string bag_file = argv[2];
//     std::string point_cloud_topic = argv[3];
//     std::string tf_topic = argv[4];
//     std::string loadcell_topic = argv[5];
//     std::string camera_topic = "/left_camera/image_raw/compressed";
//     std::string loadcell_one_topic = loadcell_topic + "_one_lbs";
//     std::string loadcell_two_topic = loadcell_topic + "_two_lbs";
//     double voxel_size = 0.1; // Adjust voxel size as needed
//     // combine the data directory with the bag file
//     bag_file = data_directory + "/" + bag_file;

// processVoxelization(bag_file, point_cloud_topic, tf_topic, loadcell_one_topic, loadcell_two_topic, camera_topic, voxel_size);
//     return 0;
// }


#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "process_voxelization.h"
#include "voxelization.h"
#include "bag_data_reader.h"
// Main function to orchestrate the voxelization process
int main(int argc, char** argv) {
    //ros::init(argc, argv, "voxelization_node");

    // Parameters
    std::string bag_file = argv[1]; 
    //"/media/marc/Seagate Backup Plus Drive/all_veg_override_rosbag_grp_4/trajectory/powerline27_loam_trajectory.bag";
    std::string point_cloud_topic = "/os_cloud_node_front/points";
    std::string tf_topic = "/tf";
    std::string loadcell_one_topic = "/loadcell_one_lbs";
    std::string loadcell_two_topic = "/loadcell_two_lbs";
    std::string camera_topic = "/left_camera/image_raw/compressed";
    double voxel_size = 0.1;

    // Initialize the ProcessVoxelization class and start processing
    ProcessVoxelization processor;
    processor.processVoxelization(bag_file, point_cloud_topic, tf_topic, loadcell_one_topic, loadcell_two_topic, camera_topic, voxel_size);

    return 0;
}

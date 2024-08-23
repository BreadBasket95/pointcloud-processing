#ifndef PROCESS_VOXELIZATION_H
#define PROCESS_VOXELIZATION_H

#include "transform_adjuster.h"
#include "bag_data_reader.h"

class ProcessVoxelization {
public:
    void processVoxelization(const std::string& bag_file, const std::string& point_cloud_topic, 
                             const std::string& tf_topic, const std::string& loadcell_one_topic, 
                             const std::string& loadcell_two_topic, const std::string& camera_topic, 
                             double voxel_size);
};


#endif // PROCESS_VOXELIZATION_H
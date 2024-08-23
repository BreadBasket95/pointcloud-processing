#ifndef TRANSFORM_ADJUSTER_H
#define TRANSFORM_ADJUSTER_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include "projected_point.h"
// Function declarations
Eigen::Matrix4f getInitialCameraToBaseLinkTransform();

// Function to project LiDAR points onto the image
void projectLiDARPoints(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud,
                        const cv::Mat& image, 
                        const cv::Mat& intrinsic_matrix, 
                        const Eigen::Matrix4f& camera_to_base_link,
                        std::vector<cv::Point2f>& points2D);

// Function to update and display the transform interactively
void updateAndDisplayTransform(int, void* userdata);

// Function to interactively adjust the transform and return the 2D projected points
std::vector<ProjectedPoint> adjustTransformInteractively(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud,
                                                      const cv::Mat& image,
                                                      const cv::Mat& intrinsic_matrix,
                                                      bool first_time_processing_);

#endif // TRANSFORM_ADJUSTER_H
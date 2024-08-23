#ifndef PROJECTED_POINT_H
#define PROJECTED_POINT_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

struct ProjectedPoint {
    cv::Point2f point2D;
    Eigen::Vector3f point3D;
    Eigen::Matrix4f updated_transform; 
};

#endif // PROJECTED_POINT_H
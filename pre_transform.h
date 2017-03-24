#ifndef PRE_TRANSFORM
#define PRE_TRANSFORM

#include <iostream>
#include "/usr/local/include/eigen3/Eigen/Core"
#include "/usr/local/include/eigen3/Eigen/Dense"
#include "/usr/local/include/eigen3/Eigen/Eigen"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZ pcl_point;

void pre_transform(float theta_init, std::vector<float> rot_ax, Eigen::Matrix4f* transform_init );

#include "pre_transform.inl"

#endif // PRE_TRANSFORM

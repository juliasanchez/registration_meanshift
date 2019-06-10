#ifndef GET_ROTATION
#define GET_ROTATION

#include <iostream>
#include "/usr/local/include/eigen3/Eigen/Core"
#include "/usr/local/include/eigen3/Eigen/Dense"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

void get_rotation(std::pair<Eigen::Vector3f, Eigen::Vector3f>& walls1, std::pair<Eigen::Vector3f, Eigen::Vector3f>& walls2, Eigen::Matrix4f* rotation_transform);

#include "get_rotation.inl"

#endif // GET_ROTATION

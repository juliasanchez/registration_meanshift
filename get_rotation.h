#ifndef GET_ROTATION
#define GET_ROTATION

#include <iostream>
#include "/usr/local/include/eigen3/Eigen/Core"
#include "/usr/local/include/eigen3/Eigen/Dense"
#include "/usr/local/include/eigen3/Eigen/Eigen"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZ pcl_point;

void get_rotation(pcl::PointCloud<pcl_point> cloud_src);

#include "get_rotation.inl"

#endif // GET_ROTATION

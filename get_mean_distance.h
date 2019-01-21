#ifndef GET_MEAN_DISTANCE
#define GET_MEAN_DISTANCE

#include <iostream>
#include <fstream>
#include "/usr/local/include/eigen3/Eigen/Core"
#include "/usr/local/include/eigen3/Eigen/Dense"
#include "/usr/local/include/eigen3/Eigen/Eigen"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

using namespace std;
typedef pcl::PointXYZ pcl_point;

void get_mean_distance(pcl::PointCloud<pcl_point>::Ptr cloud_src, pcl::KdTreeFLANN<pcl_point>::Ptr tree, Eigen::Matrix4f* transform, float* mean_distance);

#include "get_mean_distance.inl"

#endif // GET_MEAN_DISTANCE

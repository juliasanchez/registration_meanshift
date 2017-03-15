#ifndef GET_LCP
#define GET_LCP

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

void get_LCP(pcl::PointCloud<pcl_point> cloud_src, pcl::PointCloud<pcl_point> cloud_tgt, Eigen::Matrix4f* transform, int* LCP);

#include "get_LCP.inl"

#endif // GET_LCP

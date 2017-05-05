#ifndef GET_LCP
#define GET_LCP

#include <iostream>
#include <fstream>
#include "/usr/local/include/Eigen/Core"
#include "/usr/local/include/Eigen/Dense"
#include "/usr/local/include/Eigen/Eigen"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

using namespace std;
typedef pcl::PointXYZ pcl_point;

void get_LCP(pcl::PointCloud<pcl_point> cloud_src, pcl::PointCloud<pcl_point> cloud_tgt, float thresh, Eigen::Matrix4f* transform, int* LCP);

#include "get_LCP.inl"

#endif // GET_LCP

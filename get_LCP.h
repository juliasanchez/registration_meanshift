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
typedef pcl::PointNormal pcl_point;

void get_LCP(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src, pcl::KdTreeFLANN<pcl::PointNormal>::Ptr tree, float thresh, Eigen::Matrix4f* transform, float* LCP);
#include "get_LCP.inl"

#endif // GET_LCP

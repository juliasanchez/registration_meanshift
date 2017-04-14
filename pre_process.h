#ifndef PRE_PROCESS
#define PRE_PROCESS

#include <iostream>
#include <string>

#include "cloud.h"
#include "display_normals.h"

typedef pcl::PointXYZ pcl_point;

void pre_process(std::string pcd_file1,float sample,int display, pcl::PointCloud<pcl_point>::Ptr cloud_in, Eigen::Matrix4f matrix_transform, pcl::PointCloud<pcl::Normal>::Ptr);

#include "pre_process.inl"

#endif // PRE_PROCESS

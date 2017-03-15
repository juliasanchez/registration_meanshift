#ifndef GET_AXE_MAX
#define GET_AXE_MAX

#include <iostream>
#include <string>

typedef pcl::PointXYZ pcl_point;

void get_axe_max(std::string pcd_file1,float sample,float normal_radius,int display, pcl::PointCloud<pcl_point>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr);

#include "get_axe_max.inl"

#endif // GET_AXE_MAX

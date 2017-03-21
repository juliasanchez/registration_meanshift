#ifndef GET_LIM_AXIS
#define GET_LIM_AXIS

#include <iostream>
#include <string>

typedef pcl::PointXYZ pcl_point;

void get_lim_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, std::vector<float> axis, std::vector<float>& axis_lim  );

#include "get_lim_axis.inl"

#endif // GET_LIM_AXIS

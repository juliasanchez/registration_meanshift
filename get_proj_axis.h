#ifndef GET_PROJ_AXIS
#define GET_PROJ_AXIS

#include <iostream>
#include <string>

typedef pcl::PointXYZ pcl_point;

void get_proj_axis(std::vector<float> axis_lim, int axis, pcl::PointCloud<pcl_point>::Ptr cloud, std::vector<float>& hist_axis );
#include "get_proj_axis.inl"

#endif // GET_PROJ_AXIS

#ifndef GET_HIST_AXIS
#define GET_HIST_AXIS

#include <iostream>
#include <string>

typedef pcl::PointXYZ pcl_point;

void get_hist_axis(std::vector<float> axis_lim, std::vector<float> axis, pcl::PointCloud<pcl_point>::Ptr cloud, std::vector<float>& hist_axis );
#include "get_hist_axis.inl"

#endif // GET_HIST_AXIS

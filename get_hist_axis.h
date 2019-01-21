#ifndef GET_HIST_AXIS
#define GET_HIST_AXIS

#include <iostream>
#include <string>
#include "save_vector.h"
#include "norm_hist.h"

void get_hist_axis(std::vector<float>& axis_lim, std::vector<float>& axis, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool moy, std::vector<float>& hist_axis );
#include "get_hist_axis.inl"

#endif // GET_HIST_AXIS

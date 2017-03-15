#ifndef GET_NORMAL_HIST
#define GET_NORMAL_HIST

#include <iostream>
#include <string>

#include "cloud.h"
#include "display_normals.h"

typedef pcl::PointXYZ pcl_point;

void get_normal_hist(std::string pcd_file1,float sample,float normal_radius, std::vector<std::vector<float>>& hist, int display);

#include "get_normal_hist.inl"

#endif // GET_NORMAL_HIST

#ifndef GET_WALLS
#define GET_WALLS

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

typedef pcl::PointNormal pcl_point;

void get_walls(pcl::PointCloud<pcl_point>::Ptr cloud_in, float lim, std::vector<float> axis, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);

#include "get_walls.inl"

#endif // GET_WALLS

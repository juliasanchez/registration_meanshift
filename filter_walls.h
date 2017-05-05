#ifndef FILTER_WALLS
#define FILTER_WALLS

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZ pcl_point;

void filter_walls(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, std::vector<vector<double>> normals);

#include "filter_walls.inl"

#endif // GET_WALLS

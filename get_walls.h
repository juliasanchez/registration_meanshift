#ifndef GET_WALLS
#define GET_WALLS

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

typedef pcl::PointNormal pcl_point;

void get_walls(pcl::PointCloud<pcl_point>::Ptr cloud_in, float lim, Eigen::Vector3f& axis, std::set<double>& proj);

#include "get_walls.inl"

#endif // GET_WALLS

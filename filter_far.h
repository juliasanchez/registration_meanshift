#ifndef FILTER_FAR
#define FILTER_FAR

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

typedef pcl::PointNormal pcl_point;

void filter_far(pcl::PointCloud<pcl_point>::Ptr cloud_in, float thresh);

#include "filter_far.inl"

#endif // filter_far

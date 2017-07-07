#ifndef FILTER_NORMALS
#define FILTER_NORMALS

#include <iostream>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointXYZ pcl_point;

void filter_normals(pcl::PointCloud<pcl_point>::Ptr normals, float radius, float perc, std::vector< int >& ind);

#include "filter_normals.inl"

#endif // FILTER_NORMALS

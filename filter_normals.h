#ifndef FILTER_NORMALS
#define FILTER_NORMALS

#include <iostream>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointXYZ points;

void filter_normals(pcl::PointCloud<points>::Ptr normals, float radius, float perc, std::vector< int >& ind);

#include "filter_normals.inl"

#endif // FILTER_NORMALS

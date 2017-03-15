#ifndef TRANSFORM
#define TRANSFORM

#include <iostream>
#include <string>

typedef pcl::PointXYZ pcl_point;

void transform(pcl::PointCloud<pcl_point>::Ptr cloud_in, float phi, float theta);

#include "transform.inl"

#endif // TRANSFORM

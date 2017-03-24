#ifndef GET_AXIS
#define GET_AXIS

#include <iostream>
#include "MeanShift.h"

typedef pcl::PointXYZ pcl_point;

void get_axis(std::vector<Cluster> clusters, std::vector< std::vector <float> >& axis, bool* error );

#include "get_axis.inl"

#endif // GET_AXIS

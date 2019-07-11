#ifndef DENSITY_FILTER
#define DENSITY_FILTER

#include <iostream>
#include <fstream>
#include <cstdlib>
#include "/usr/local/include/eigen3/Eigen/Core"
#include "/usr/local/include/eigen3/Eigen/Dense"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <map>
#include <vector>
#include "/usr/local/include/eigen3/Eigen/Core"
#include "/usr/local/include/eigen3/Eigen/Dense"
#include "/usr/local/include/eigen3/Eigen/Eigenvalues"
#include "save_points.h"


using namespace std;
typedef pcl::PointXYZ points;

void density_filter(pcl::PointCloud<points>::Ptr cloud_in, float angle_uncertainty_small, float angle_uncertainty_large, int keep);

#include "density_filter.inl"

#endif // DENSITY_FILTER

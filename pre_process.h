#ifndef PRE_PROCESS
#define PRE_PROCESS

#include <iostream>
#include <string>
#include <chrono>

#include "cloud.h"

typedef pcl::PointNormal pcl_point;

void pre_process(std::string pcd_file,float sample, float normal_radius, float far, cloud<pcl_point> *cloud_in, double* reso);

#include "pre_process.inl"

#endif // PRE_PROCESS

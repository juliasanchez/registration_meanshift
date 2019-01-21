#ifndef PCN2PC
#define PCN2PC

#include<iostream>

typedef pcl::PointNormal pcl_point;

void pcn2pc(pcl::PointCloud<pcl_point>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout);

#include "pcn2pc.inl"

#endif // PCN2PC

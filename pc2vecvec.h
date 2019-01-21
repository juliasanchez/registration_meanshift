#ifndef PC2VECVEC
#define PC2VECVEC

#include<iostream>

typedef pcl::PointNormal pcl_point;

void pc2vecvec(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, std::vector< vector<double> >& vec);

#include "pc2vecvec.inl"

#endif // PC2VECVEC

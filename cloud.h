#ifndef CLOUD_H
#define CLOUD_H

#include <vector>
#include <string>
#include <iostream>

// PCL INCLUDES

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/octree/octree_pointcloud.h>
#define EIGEN_USE_NEW_STDVECTOR
#include "/usr/local/include/eigen3/Eigen/StdVector"

#include "filter_far.h"

class cloud
{
public:
    cloud();
    void setInputCloud(typename pcl::PointCloud<pcl::PointNormal>::Ptr);
    void setTree();
    void getScale(float*);
    void clean(float far);
    typename pcl::PointCloud<pcl::PointNormal>::Ptr getInputCloud();

    void getNormals(float);
    int getSize() const;
    void load (std::string);
    double computeCloudResolution ();
    void sample(float samp);
    void rand_sample(float samp);
    void orient() const;


private:
    typename pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in;
    typename pcl::search::KdTree<pcl::PointNormal> tree;
};

#include "cloud.inl"

#endif // CLOUD

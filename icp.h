#ifndef ICP
#define ICP

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection.h>

void icp(pcl::PointCloud<pcl::PointNormal>::Ptr cloud1, pcl::PointCloud<pcl::PointNormal>::Ptr cloud2, float RANSAC_thresh, float rej_dist, int it, Eigen::Matrix4f *transformation);

#include "icp.inl"

#endif // ICP

void get_rotation(std::pair<Eigen::Vector3f, Eigen::Vector3f>& walls1, std::pair<Eigen::Vector3f, Eigen::Vector3f>& walls2, Eigen::Matrix4f* rot_transfo )
{
    pcl::registration::TransformationEstimationSVD<pcl_point, pcl_point> estimator;
    pcl::PointCloud<pcl_point>::Ptr cloud_clus1(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl_point>::Ptr cloud_clus2(new pcl::PointCloud<pcl_point>);
    cloud_clus1->width    = 3;
    cloud_clus1->height   = 1;
    cloud_clus1->is_dense = false;
    cloud_clus1->points.resize (cloud_clus1->width * cloud_clus1->height);
    *cloud_clus2=*cloud_clus1;

    cloud_clus1->points[0].x=walls1.first(0);
    cloud_clus1->points[0].y=walls1.first(1);
    cloud_clus1->points[0].z=walls1.first(2);
    cloud_clus1->points[1].x=walls1.second(0);
    cloud_clus1->points[1].y=walls1.second(1);
    cloud_clus1->points[1].z=walls1.second(2);
    cloud_clus1->points[2].x=0;
    cloud_clus1->points[2].y=0;
    cloud_clus1->points[2].z=0;

    cloud_clus2->points[0].x=walls2.first(0);
    cloud_clus2->points[0].y=walls2.first(1);
    cloud_clus2->points[0].z=walls2.first(2);
    cloud_clus2->points[1].x=walls2.second(0);
    cloud_clus2->points[1].y=walls2.second(1);
    cloud_clus2->points[1].z=walls2.second(2);
    cloud_clus2->points[2].x=0;
    cloud_clus2->points[2].y=0;
    cloud_clus2->points[2].z=0;


    estimator.estimateRigidTransformation(*cloud_clus1, *cloud_clus2, *rot_transfo);
    (*rot_transfo)(0,3)=0;
    (*rot_transfo)(1,3)=0;
    (*rot_transfo)(2,3)=0;
}

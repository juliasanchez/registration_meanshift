void get_rotation(std::vector<std::vector<double>> walls1, std::vector<std::vector<double>> walls2, Eigen::Matrix4f* rotation_transform )
{
    Eigen::Matrix4f transformation= Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotation_transform0 = Eigen::Matrix4f::Identity();

    pcl::registration::TransformationEstimationSVD<pcl_point, pcl_point> estimator;
    pcl::PointCloud<pcl_point>::Ptr cloud_clus1(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl_point>::Ptr cloud_clus2(new pcl::PointCloud<pcl_point>);
    cloud_clus1->width    = 3;
    cloud_clus1->height   = 1;
    cloud_clus1->is_dense = false;
    cloud_clus1->points.resize (cloud_clus1->width * cloud_clus1->height);
    *cloud_clus2=*cloud_clus1;

    for (int i=0; i<2; i++)
    {
        cloud_clus1->points[i].x=walls1[i][0];
        cloud_clus1->points[i].y=walls1[i][1];
        cloud_clus1->points[i].z=walls1[i][2];
        cloud_clus2->points[i].x=walls2[i][0];
        cloud_clus2->points[i].y=walls2[i][1];
        cloud_clus2->points[i].z=walls2[i][2];
    }

    cloud_clus1->points[2].x=0;
    cloud_clus1->points[2].y=0;
    cloud_clus1->points[2].z=0;
    cloud_clus2->points[2].x=0;
    cloud_clus2->points[2].y=0;
    cloud_clus2->points[2].z=0;


    estimator.estimateRigidTransformation(*cloud_clus1, *cloud_clus2, transformation);
    rotation_transform0=transformation;
    rotation_transform0(0,3)=0;
    rotation_transform0(1,3)=0;
    rotation_transform0(2,3)=0;
    *rotation_transform=rotation_transform0;
}
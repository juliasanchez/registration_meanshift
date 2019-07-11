void pcn2pc(pcl::PointCloud<pcl::PointNormal>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_subsampled(new pcl::PointCloud<pcl::PointNormal>);
    *cloud_subsampled = *cloudin;

    cloudout->width    = cloud_subsampled->points.size();
    cloudout->height   = 1;
    cloudout->is_dense = false;
    cloudout->points.resize (cloud_subsampled->width * cloud_subsampled->height);


    for (int i=0; i<cloud_subsampled->points.size(); i++)
    {
//        float plane_dist = abs(cloud_subsampled->points[i].normal_x * cloud_subsampled->points[i].x + cloud_subsampled->points[i].normal_y * cloud_subsampled->points[i].y + cloud_subsampled->points[i].normal_z * cloud_subsampled->points[i].z);
        cloudout->points[i].x = cloud_subsampled->points[i].normal_x;
        cloudout->points[i].y = cloud_subsampled->points[i].normal_y;
        cloudout->points[i].z = cloud_subsampled->points[i].normal_z;
    }
}

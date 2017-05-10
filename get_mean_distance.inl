void get_mean_distance(pcl::PointCloud<pcl_point> cloud_src, pcl::PointCloud<pcl_point> cloud_tgt, Eigen::Matrix4f* transform, float* mean_distance)
{
    //transform
    pcl::transformPointCloud (cloud_src, cloud_src, *transform);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    pcl::PointCloud<pcl_point>::Ptr cloud_src_ptr(new pcl::PointCloud<pcl_point>);
    *cloud_src_ptr=cloud_src;
    tree.setInputCloud(cloud_src_ptr);

    //compute mean_distance

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    int n=0;
    *mean_distance=0;

    for (int k=0; k<cloud_tgt.size(); k++)
    {

        if ( tree.nearestKSearch (cloud_tgt.points[k], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            if(sqrt(pointNKNSquaredDistance[0])<0.1)
            {
                *mean_distance=*mean_distance+sqrt(pointNKNSquaredDistance[0]);
                n++;
            }
        }
    }


    *mean_distance=*mean_distance/n;
}

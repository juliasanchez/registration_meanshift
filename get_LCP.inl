void get_LCP(pcl::PointCloud<pcl_point> cloud_src, pcl::PointCloud<pcl_point> cloud_tgt, Eigen::Matrix4f* transform, int* LCP)
{
    float temp=0;
    int idx;

    //transform
    pcl::transformPointCloud (cloud_src, cloud_src, *transform);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    pcl::PointCloud<pcl_point>::Ptr cloud_src_ptr(new pcl::PointCloud<pcl_point>);
    *cloud_src_ptr=cloud_src;
    tree.setInputCloud(cloud_src_ptr);

    //compute LCP

    *LCP=0;

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    for (int k=0; k<cloud_tgt.size(); k++)
    {

        if ( tree.nearestKSearch (cloud_tgt.points[k], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            if(pointNKNSquaredDistance[0]<0.02)
            {
                *LCP=*LCP+1;
            }
        }
    }
}

void get_LCP(pcl::PointCloud<pcl_point>::Ptr cloud_src, pcl::KdTreeFLANN<pcl_point>::Ptr tree, float thresh, Eigen::Matrix4f* transform, int* LCP)
{
    //transform
    pcl::PointCloud<pcl_point> cloud_src_transformed;
    pcl::transformPointCloud (*cloud_src, cloud_src_transformed, *transform);

    //compute LCP

    *LCP=0;
    int n=0;

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    for (int k=0; k<cloud_src_transformed.size(); k++)
    {

        if ( tree->nearestKSearch (cloud_src_transformed.points[k], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            if(sqrt(pointNKNSquaredDistance[0])<thresh)
            {
                *LCP=*LCP+1;
                n++;
            }
        }
    }
}

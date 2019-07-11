void get_LCP(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src, pcl::KdTreeFLANN<pcl::PointNormal>::Ptr tree, float thresh, Eigen::Matrix4f* transform, float* LCP)
{
//    thresh = 0.1;
    //transform
    pcl::PointCloud<pcl_point> cloud_src_transformed;
    pcl::transformPointCloud (*cloud_src, cloud_src_transformed, *transform);

    //compute LCP

    *LCP=0;

    std::vector<int> pointIdxNKNSearch(2);
    std::vector<float> pointNKNSquaredDistance(2);

//    for (int k=0; k<cloud_src_transformed.size(); k++)
//    {
//        if ( tree->nearestKSearch (cloud_src_transformed.points[k], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//        {
//            if(sqrt(pointNKNSquaredDistance[0])<thresh)
//                ++(*LCP);
//        }
//    }

    for (int k=0; k<cloud_src_transformed.size(); k++)
    {
        tree->nearestKSearch (cloud_src_transformed.points[k], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        float dist_nn_target_source = sqrt(pointNKNSquaredDistance[0]);
//        tree->nearestKSearch (tree->getInputCloud()->points[pointIdxNKNSearch[0]], 2, pointIdxNKNSearch, pointNKNSquaredDistance);
//        float dist_nn_target_target = sqrt(pointNKNSquaredDistance[1]);
        if(dist_nn_target_source < thresh)//dist_nn_target_target)
            (*LCP) += exp(-dist_nn_target_source/thresh);//dist_nn_target_target);
    }
}

void density_filter(pcl::PointCloud<pcl_point>::Ptr cloud_in, float thresh, int points)
{
    pcl::KdTreeFLANN<pcl_point> tree;
    tree.setInputCloud(cloud_in);

    pcl::PointCloud<pcl_point> cloud_out;
    std::vector<int> pointIdxNKNSearch(points+1);
    std::vector<float> pointNKNSquaredDistance(points+1);

    for (int i=0; i< cloud_in->size(); i++)
    {
        if ( tree.nearestKSearch (cloud_in->points[i], points+1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            if(sqrt(pointNKNSquaredDistance[points])<thresh)
            {
                pcl_point point;
                point.x=cloud_in->points[i].x;
                point.y=cloud_in->points[i].y;
                point.z=cloud_in->points[i].z;
                cloud_out.points.push_back(point);
            }
        }
    }

    cloud_out.width = (uint32_t)  cloud_out.points.size();
    cloud_out.height = 1;
    *cloud_in=cloud_out;
}

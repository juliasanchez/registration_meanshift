void filter_far(pcl::PointCloud<pcl_point>::Ptr cloud_in, float thresh)
{
    pcl::PointCloud<pcl_point>::Ptr cloud_filtered(new pcl::PointCloud<pcl_point>);

    for (int i=0; i<cloud_in->points.size(); i++)
    {
       float dist=sqrt(cloud_in->points[i].x*cloud_in->points[i].x + cloud_in->points[i].y*cloud_in->points[i].y +cloud_in->points[i].z*cloud_in->points[i].z);
       if(dist<thresh)
       {
           cloud_filtered->points.push_back(cloud_in->points[i]);
       }
    }
    cloud_filtered->width = (uint32_t)  cloud_filtered->points.size();
    cloud_filtered->height = 1;
    pcl::copyPointCloud(*cloud_filtered,*cloud_in);
}


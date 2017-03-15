void get_walls(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, float lim, std::vector<float> axis, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    float dot;

    for (int i=0; i<cloud_in->points.size(); i++)
    {

        dot=cloud_in->points[i].normal_x*axis[0]+cloud_in->points[i].normal_y*axis[1]+cloud_in->points[i].normal_z*axis[2];


        if( abs(dot)>lim )
        {
            pcl::PointXYZ point;
            point.x=cloud_in->points[i].x;
            point.y=cloud_in->points[i].y;
            point.z=cloud_in->points[i].z;
            cloud_filtered->points.push_back(point);
        }
    }
    cloud_filtered->width = (uint32_t)  cloud_filtered->points.size();
    cloud_filtered->height = 1;
}


void filter_walls(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, std::vector<vector<double>> normals)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
    std::vector<float> dot(normals.size());

    for (int i=0; i<cloud_in->points.size(); i++)
    {
        for(int j=0; j<normals.size(); j++)
        {
            dot[j]=cloud_in->points[i].normal_x*normals[j][0]+cloud_in->points[i].normal_y*normals[j][1]+cloud_in->points[i].normal_z*normals[j][2];
            if( abs(dot[j])>0.98)
            {
                pcl::PointNormal point;
                point.x=cloud_in->points[i].x;
                point.y=cloud_in->points[i].y;
                point.z=cloud_in->points[i].z;
                point.normal_x=cloud_in->points[i].normal_x;
                point.normal_y=cloud_in->points[i].normal_y;
                point.normal_z=cloud_in->points[i].normal_z;
                cloud_filtered->points.push_back(point);
            }
        }

    }
    cloud_filtered->width = (uint32_t)  cloud_filtered->points.size();
    cloud_filtered->height = 1;
    pcl::copyPointCloud(*cloud_filtered,*cloud_in);
}


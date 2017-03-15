void pre_process(std::string pcd_file,float sample,float normal_radius, int display, int level, pcl::PointCloud<pcl_point>::Ptr cloud_in, Eigen::Matrix4f matrix_transform, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    cloud<pcl_point> cloud_src;
    cloud_src.setInputCloud(cloud_in);
    cloud_src.load(pcd_file);
    cloud_src.sample(sample);
    cloud_src.clean();
    cloud_src.transform(matrix_transform);

    cloud_src.getNormals(normal_radius, normals);

    if (display)
    {
        pcl::PointCloud<pcl_point>::Ptr cloud0(new pcl::PointCloud<pcl_point>);

        cloud0->width    = cloud_in->width;
        cloud0->height   = cloud_in->height;
        cloud0->is_dense = cloud_in->is_dense;
        cloud0->points.resize (cloud0->width * cloud0->height);

        for (size_t i = 0; i < cloud0->points.size (); ++i)
        {
            cloud0->points[i].x = 0;
            cloud0->points[i].y = 0;
            cloud0->points[i].z = 0;
        }

        if (display==1)
        {
            display_normals(cloud0, normals,2);
        }
        else if (display==2)
        {
            display_normals(cloud_in, normals, 2);
        }


    }
}

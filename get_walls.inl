void get_walls(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, float lim, Eigen::Vector3f& axis, std::set<double>& proj)
{
    float dot;

    for (int i=0; i<cloud_in->points.size(); i++)
    {
        dot=cloud_in->points[i].normal_x*axis(0)+cloud_in->points[i].normal_y*axis(1)+cloud_in->points[i].normal_z*axis(2);

        if( abs(dot)>lim )
            proj.insert(cloud_in->points[i].x*axis(0) + cloud_in->points[i].y*axis(1)+cloud_in->points[i].z*axis(2));
    }
}


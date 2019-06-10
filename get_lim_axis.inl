void get_lim_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, Eigen::Vector3f& axis, std::vector<float>& axis_lim  )
{
    float v_axis_src;
    float v_axis_tgt;

    for (size_t i = 0; i < cloud_src->points.size (); ++i)
    {
        v_axis_src = cloud_src->points[i].x*axis(0) + cloud_src->points[i].y*axis(1)+cloud_src->points[i].z*axis(2);
        if( axis_lim[1]<v_axis_src )
        {
                axis_lim[1]=v_axis_src;
        }
        if(axis_lim[0]>v_axis_src)
        {
                axis_lim[0]=v_axis_src;
        }
    }

    for (size_t i = 0; i < cloud_tgt->points.size (); ++i)
    {
        v_axis_tgt = cloud_tgt->points[i].x*axis(0)+ cloud_tgt->points[i].y*axis(1)+cloud_tgt->points[i].z*axis(2);
        if( axis_lim[1]<v_axis_tgt )
        {
                axis_lim[1]=v_axis_tgt;
        }
        if(axis_lim[0]>v_axis_tgt)
        {
                axis_lim[0]=v_axis_tgt;
        }
    }

    axis_lim[1]=axis_lim[1]+0.003*(axis_lim[1]-axis_lim[0]);
    axis_lim[0]=axis_lim[0]-0.003*(axis_lim[1]-axis_lim[0]);

}

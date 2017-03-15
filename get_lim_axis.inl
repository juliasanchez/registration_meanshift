void get_lim_axis(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt, std::vector<std::vector<float>> axis, std::vector<std::vector<float>>& axis_lim  )
{
    float v_axis_src;
    float v_axis_tgt;

    for (int k = 0; k < 3; ++k)
    {
        for (size_t i = 0; i < cloud_src->points.size (); ++i)
        {
            v_axis_src = cloud_src->points[i].x*axis[k][0] + cloud_src->points[i].y*axis[k][1]+cloud_src->points[i].z*axis[k][2];
            if( axis_lim[k][0]<v_axis_src )
            {
                    axis_lim[k][0]=v_axis_src;
            }
            if(axis_lim[k][1]>v_axis_src)
            {
                    axis_lim[k][1]=v_axis_src;
            }
        }

        for (size_t i = 0; i < cloud_tgt->points.size (); ++i)
        {
            v_axis_tgt = cloud_tgt->points[i].x*axis[k][0]+ cloud_tgt->points[i].y*axis[k][1]+cloud_tgt->points[i].z*axis[k][2];
            if( axis_lim[k][0]<v_axis_tgt )
            {
                    axis_lim[k][0]=v_axis_tgt;
            }
            if(axis_lim[k][1]>v_axis_tgt)
            {
                    axis_lim[k][1]=v_axis_tgt;
            }
        }
    }

}

void get_hist_axis(std::vector<float> axis_lim, std::vector<float> axis, pcl::PointCloud<pcl_point>::Ptr cloud, std::vector<float>& hist_axis )
{
    int N_hist_axis=hist_axis.size();

    float v_axis;

    float delta=(float)(  (axis_lim[1]-axis_lim[0])/(float)(N_hist_axis)  );


    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        v_axis = cloud->points[i].x*axis[0] + cloud->points[i].y*axis[1]+cloud->points[i].z*axis[2];
        int n=(int)(  (v_axis-axis_lim[0]) /delta  );
        hist_axis[n]=hist_axis[n]+1.0/(float)(cloud->points.size ());
    }
}

void get_proj_axis(std::vector<float> axis_lim, int axis, pcl::PointCloud<pcl_point>::Ptr cloud, std::vector<float>& hist_axis )
{
      int N_hist_axis=hist_axis.size();

      float v_axis;

      float delta=(float)(  (axis_lim[1]-axis_lim[0])/(float)(N_hist_axis)  );

      switch(axis)
      {
        case 0:
            for (size_t i = 0; i < cloud->points.size (); ++i)
            {
              v_axis = cloud->points[i].x;
              int n=(int)(  (v_axis-axis_lim[0]) /delta  );
              hist_axis[n]=hist_axis[n]+1.0/(float)(cloud->points.size ());
            }
            break;

        case 1:
            for (size_t i = 0; i < cloud->points.size (); ++i)
            {
                v_axis = cloud->points[i].y;
                int n=(int)(  (v_axis-axis_lim[0]) /delta  );
                hist_axis[n]=hist_axis[n]+1.0/(float)(cloud->points.size ());
            }
            break;


        case 2:
            for (size_t i = 0; i < cloud->points.size (); ++i)
            {
                v_axis = cloud->points[i].z;
                int n=(int)(  (v_axis-axis_lim[0]) /delta  );
                hist_axis[n]=hist_axis[n]+1.0/(float)(cloud->points.size ());
            }
            break;

        default:
            cout<<"Error, bad axis\n";
            break;
      }
}

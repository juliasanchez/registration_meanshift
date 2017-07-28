void get_hist_axis(std::vector<float> axis_lim, std::vector<float> axis, pcl::PointCloud<pcl_point>::Ptr cloud, std::vector<float>& hist_axis )
{
    int N_hist_axis=hist_axis.size();

    float v_axis;

    float delta=(float)(axis_lim[1]-axis_lim[0]) / (float)(N_hist_axis);


    for (size_t i = 0; i < cloud->points.size (); ++i)
    {        
        v_axis = cloud->points[i].x*axis[0] + cloud->points[i].y*axis[1]+cloud->points[i].z*axis[2];
        int n=(int)(  (v_axis-axis_lim[0]) /delta  );
        hist_axis[n]=hist_axis[n]+1.0/(float)(cloud->points.size ());
    }

//    save_vector (hist_axis, "hist_axis1.csv");

//    int neigh=(int)(0.01/delta);
//    float cumul=0;
//    std::vector<float> hist0(hist_axis.size(),0.0);

//    for(int i=0; i<neigh+1; i++)
//    {
//        cumul=0;
//        for(int k=0; k<i+neigh+1; k++)
//        {
//            cumul+=hist_axis[k];
//        }
//        hist0[i]=cumul/(i+neigh+1);
//    }


//   for(int i=neigh+1; i<hist_axis.size()-neigh; i++)
//   {
//       cumul+=hist_axis[i+neigh];
//       cumul-=hist_axis[i-neigh-1];
//       hist0[i]=cumul/(2*neigh+1);
//   }

//   for(int i=hist_axis.size()-neigh; i<hist_axis.size(); i++)
//   {
//       cumul=0;
//       for(int k=i-neigh; k<hist_axis.size(); k++)
//       {
//           cumul+=hist_axis[k];
//       }
//       hist0[i]=cumul/(hist_axis.size()-i+neigh);
//   }

//   hist_axis=hist0;
//   save_vector (hist_axis, "hist_axis2.csv");


}

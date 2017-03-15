void get_normal_hist(std::string pcd_file1,float sample,float normal_radius, std::vector<std::vector<float>>& hist, int display)
{

    int n=1;
    n++;

    int N_hist=hist.size();
    pcl::PointCloud<pcl_point>::Ptr cloud_in(new pcl::PointCloud<pcl_point>);

    cloud<pcl_point> cloud_src;
    cloud_src.setInputCloud(cloud_in);
    cloud_src.load(pcd_file1);
    cloud_src.sample(sample);
    cloud_src.clean();


    pcl::io::savePCDFileASCII ("sampled_cloud.pcd", *cloud_in);
    pcl::search::KdTree<pcl_point>::Ptr tree (new pcl::search::KdTree<pcl_point>);
    cloud_src.getTree(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
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

      display_normals(cloud0, normals);
    }

      float theta;
      float phi;
      float z_dot=normals->points[0].normal_z;
      float x_dot=1;

      for (int n=0; n<N_hist; n++)
      {
          for(int m=0; m<N_hist*2; m++)
          {
              hist[n][m]=0;
          }
      }

      int done=0;
      float count=0;

      for (size_t i = 0; i < normals->points.size (); ++i)
      {
          float z_dot = normals->points[i].normal_z /sqrt(normals->points[i].normal_x*normals->points[i].normal_x+normals->points[i].normal_y*normals->points[i].normal_y+normals->points[i].normal_z*normals->points[i].normal_z);
          theta=acos(z_dot);
          float x_dot = normals->points[i].normal_x /sqrt(normals->points[i].normal_x*normals->points[i].normal_x+normals->points[i].normal_y*normals->points[i].normal_y);
          phi=acos(x_dot);
          if(normals->points[i].normal_y<0 && phi!=0)
          {
              phi=2*M_PI-phi;
          }
          done=0;
          for (int n = 0; n < N_hist; ++n)
          {
              if (  ((n+1)*M_PI/N_hist-theta)>0 && theta-n*M_PI/N_hist>=0 )
              {
                  for (int m = 0; m < N_hist*2; ++m)
                  {
                      if (   ((m+1)*M_PI/N_hist-phi)>0 && phi-m*M_PI/N_hist>=0    )
                      {
                      hist[n][m]=hist[n][m]+1.0/(float)(normals->points.size ());
                      done=1;
                      count=count +1.0/(float)(normals->points.size ());
                      break;
                      }
                      else if (m==N_hist*2-1)
                      {
                          std::cout<<"can't order this normal :  x:"<<x_dot<<"   z: "<<z_dot<<std::endl<<std::endl;
                      }
                  }
              }
              else if (n==N_hist-1 && done==0)
              {
                  std::cout<<"can't order this normal :  indice:"<<i<<"   x: "<<x_dot<<"   z: "<<z_dot<<std::endl<<std::endl;
              }
              if(done)
              {break;}
           }
      }
}

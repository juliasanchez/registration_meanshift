void get_axis(std::vector<Cluster> clusters, std::vector< std::vector <float> >& axis, bool* error )
{
    *error=false;
    int temp1=0;
    int temp2=0;

    std::vector<double> norm1(3);
    std::vector<double> norm2(3);
    std::vector<double> norm3(3);

    for (int i=0; i<clusters.size(); i++)
    {
        double dot1=norm1[0]*clusters[i].mode[0]+norm1[1]*clusters[i].mode[1]+norm1[2]*clusters[i].mode[2];
        double dot2=norm2[0]*clusters[i].mode[0]+norm2[1]*clusters[i].mode[1]+norm2[2]*clusters[i].mode[2];
        if(clusters[i].original_points.size()>temp1 && abs(clusters[i].mode[2])<0.7) //hypothesis 1 : roof with theta<45°
        {
          if(abs(dot1)<0.90)
          {
              temp2=temp1;
              norm2=norm1;
          }
          temp1=clusters[i].original_points.size();
          norm1=clusters[i].mode;

        }
        else if(clusters[i].original_points.size()>temp2 && abs(dot2)<0.90 && abs(dot1)<0.90 && abs(clusters[i].mode[2])<0.7 )
        {
          temp2=clusters[i].original_points.size();
          norm2=clusters[i].mode;
        }
    }

    if(temp1==0 || temp2==0)
    {
        *error=true;
    }

    norm3[0]=norm1[1]*norm2[2]-norm1[2]*norm2[1];
    norm3[1]=norm1[2]*norm2[0]-norm1[0]*norm2[2];
    norm3[2]=norm1[0]*norm2[1]-norm1[1]*norm2[0]; //hypothesis 2 : walls perpendicular to roof and ground

    float modu=sqrt(norm3[0]*norm3[0]+norm3[1]*norm3[1]+norm3[2]*norm3[2]);

    for (int i=0; i<3; i++)
    {
        axis[0][i]=norm1[i]/modu;
        axis[1][i]=norm2[i]/modu;
        axis[2][i]=norm3[i]/modu;
    }
}

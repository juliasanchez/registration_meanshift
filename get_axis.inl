void get_axis(std::vector<Cluster> clusters, std::vector< std::vector <float> >& axis, bool* error )
{
    *error=false;

    std::vector<double> norm1(3);
    std::vector<double> norm2(3);
    std::vector<double> norm3(3);

    //first axis is the cluster with more points

    norm1=clusters[0].mode;

    //second axis is the cluster with more points after axis1 which is not parallel to axis1
    int k=1;
    int p2=0;
    while (k<clusters.size() && p2==0)
    {
        double dot1=norm1[0]*clusters[k].mode[0]+norm1[1]*clusters[k].mode[1]+norm1[2]*clusters[k].mode[2];
        if(abs(dot1)<0.9)
        {
            norm2=clusters[k].mode;
            p2=1;
        }
        k++;
    }

    //third axis is the cluster with more points after axis1 and axis2 which is not parallel to axis1 nor axis2
    int p3=0;
    if(p2)
    {
        while (k<clusters.size() && p3==0)
        {
            double dot2=norm2[0]*clusters[k].mode[0]+norm2[1]*clusters[k].mode[1]+norm2[2]*clusters[k].mode[2];
            double dot1=norm1[0]*clusters[k].mode[0]+norm1[1]*clusters[k].mode[1]+norm1[2]*clusters[k].mode[2];
            if(abs(dot1)<0.9 && abs(dot2)<0.9)
            {
                norm3=clusters[k].mode;
                p3=1;
            }
             k++;
        }       
    }

    if(p2==0 || p3==0)
    {
        *error=true;
    }

    for (int i=0; i<3; i++)
    {
        axis[0][i]=norm1[i];
        axis[1][i]=norm2[i];
        axis[2][i]=norm3[i];
    }

}

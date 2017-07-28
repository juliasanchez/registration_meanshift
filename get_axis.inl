void get_axis(std::vector<Cluster> clusters, std::vector< std::vector <float> >& axis, bool* error )
{
    *error=false;

    //third axis is the cluster with more points after axis1 and axis2 which is not parallel to axis1 nor axis2
    int p=0;
    int k=0;
    while (k<clusters.size() && p==0)
    {
        double dot2=axis[1][0]*clusters[k].mode[0]+axis[1][1]*clusters[k].mode[1]+axis[1][2]*clusters[k].mode[2];
        double dot1=axis[0][0]*clusters[k].mode[0]+axis[0][1]*clusters[k].mode[1]+axis[0][2]*clusters[k].mode[2];

        std::vector<float> cross (3);
        cross[0]=axis[0][1]*axis[1][2]-axis[0][2]*axis[1][1];
        cross[1]=-axis[0][0]*axis[1][2]+axis[0][2]*axis[1][0];
        cross[2]=axis[0][0]*axis[1][1]-axis[0][1]*axis[1][0];

        double dot3=clusters[k].mode[0]*cross[0]+clusters[k].mode[1]*cross[1]+clusters[k].mode[2]*cross[2];

        if(abs(dot1)<0.9 && abs(dot2)<0.9 && abs(dot3)>0.1) //condition : axis z not parallel to axis x, not parallel to axis
        {
            for (int i=0; i<3; i++)
            {
                axis[2][i]=clusters[k].mode[i];
            }
            p=1;
        }
        k++;
    }

    if(p==0)
    {
        *error=true;
    }

}

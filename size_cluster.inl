float dot(std::vector<double> v1, std::vector<double> v2)
{
    return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
}

float size_cluster(Cluster clus)
{
    float temp=10;
    for(int i=0; i<clus.original_points.size(); i++)
    {
        for(int j=0; j<clus.original_points.size(); j++)
        {
            float scal=dot(clus.original_points[i],clus.original_points[j]);
            if(scal<temp)
               temp=scal;
        }
    }

    return temp;
}




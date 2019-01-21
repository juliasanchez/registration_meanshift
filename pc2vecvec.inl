void pc2vecvec(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, std::vector< vector<double> >& vec)
{
    vec.resize(cloudin->points.size());
    for (int i=0; i<cloudin->points.size(); i++)
    {
        vec[i].resize(3);
        vec[i][0]=cloudin->points[i].x;
        vec[i][1]=cloudin->points[i].y;
        vec[i][2]=cloudin->points[i].z;
    }

}

void get_axe_max(pcl::PointCloud<pcl_point>::Ptr cloud_in, float* max_x, float* max_y)
{
    pcl::points minPt, maxPt;
    pcl::getMinMax3D (*cloud_in, minPt, maxPt);
    max_x=maxPt.x-minPt.x;
}

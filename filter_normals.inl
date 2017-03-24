void filter_normals(pcl::PointCloud<pcl_point>::Ptr normals, float radius, float perc)
{
    pcl::RadiusOutlierRemoval<pcl_point> rorfilter (true); // Initializing with true will allow us to extract the removed indices
    rorfilter.setInputCloud (normals);
    rorfilter.setRadiusSearch (radius);
    rorfilter.setMinNeighborsInRadius (floor(normals->points.size()*perc )  );
    rorfilter.filter (*normals);
}

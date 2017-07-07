void filter_normals(pcl::PointCloud<pcl_point>::Ptr normals, float radius, float perc, std::vector< int >& ind)
{
    pcl::RadiusOutlierRemoval<pcl_point> rorfilter;
    rorfilter.setInputCloud (normals);
    rorfilter.setRadiusSearch (radius);
    rorfilter.setMinNeighborsInRadius (floor(normals->points.size()*perc )  );
    rorfilter.filter (*normals);
    rorfilter.filter(ind);
}

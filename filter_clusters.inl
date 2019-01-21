void filter_clusters(std::vector<Cluster>& clusters)
{
    for (int i=0; i<clusters.size(); i++)
    {
        float size=size_cluster(clusters[i]);
        if(size<0.97)
        {
            clusters[i].original_points.resize(0);
        }
    }
}

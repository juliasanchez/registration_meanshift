void save_clusters(std::vector<Eigen::Vector3f>& clusters, std::string Name_model)
{
    for(int clus = 0; clus < clusters.size(); clus++)
    {
        std::stringstream sstm;
        sstm<<Name_model<<clus<<".csv";
        std::string cluster_name = sstm.str();
        save_cluster(clusters[clus],cluster_name);
    }
}

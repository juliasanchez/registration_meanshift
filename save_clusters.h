#ifndef SAVE_CLUSTERS
#define SAVE_CLUSTERS

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "save_cluster.h"

void save_clusters(std::vector<Eigen::Vector3f>& clusters, std::string Name_model);

#include "save_clusters.inl"

#endif // SAVE_CLUSTERS

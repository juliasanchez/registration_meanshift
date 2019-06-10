#ifndef SAVE_CLUSTER
#define SAVE_CLUSTER

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

void save_cluster(Eigen::Vector3f& vec, std::string file_name);

#include "save_cluster.inl"

#endif // SAVE_CLUSTER

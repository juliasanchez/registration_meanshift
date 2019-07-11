#ifndef SAVE_POINTS
#define SAVE_POINTS

#include <iostream>
#include <string>

void save_points(std::vector<Eigen::Vector3f>& vec, std::string file_name);

#include "save_points.inl"

#endif // save_points

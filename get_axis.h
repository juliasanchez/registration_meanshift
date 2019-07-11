#ifndef GET_AXIS
#define GET_AXIS

#include <iostream>
#include <vector>

void get_axis(std::vector<Eigen::Vector3f> &clusters1, std::vector<Eigen::Vector3f> &clusters2, std::vector< Eigen::Vector3f >& axis);

#include "get_axis.inl"

#endif // GET_AXIS

#ifndef GET_LIM_AXIS
#define GET_LIM_AXIS

#include <iostream>
#include <string>
#include <set>

void get_lim_axis(std::set<double> proj_src, std::set<double> proj_tgt, std::vector<float>& axis_lim  );

#include "get_lim_axis.inl"

#endif // GET_LIM_AXIS

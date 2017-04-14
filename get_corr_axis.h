#ifndef GET_CORR_AXIS
#define GET_CORR_AXIS

#include <iostream>
#include <string>
#include "save_vector.h"
#include "envelope.h"

void get_corr_axis(std::vector<float>& hist1_axisi, std::vector<float>& hist2_axisi, std::vector<float>& error_axis, int *translation_axis);

#include "get_corr_axis.inl"

#endif // GET_CORR_AXIS

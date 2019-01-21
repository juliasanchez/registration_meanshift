#ifndef EXTRACT_SPOTS
#define EXTRACT_SPOTS

#include <iostream>
#include <string>

void extract_spots(std::vector<std::vector<float>>& hist_axis, std::vector<std::vector<float>>& spots, float thresh  );

#include "extract_spots.inl"

#endif // EXTRACT_SPOTS

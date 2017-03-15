#ifndef SAVE_NORMALS
#define SAVE_NORMALs

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

void save_normals(std::vector<std::vector<double>>& mat, std::string file_name);

#include "save_normals.inl"

#endif // SAVE_NORMALS

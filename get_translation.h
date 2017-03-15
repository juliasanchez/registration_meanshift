#ifndef GET_TRANSLATION
#define GET_TRANSLATION

#include "get_proj_axis.h"
#include "get_hist_axis.h"
#include "get_corr_axis.h"
#include "save_vector.h"
#include "get_walls.h"
#include "norm_hist.h"
#include "transform.h"
#include "envelope.h"
#include "get_lim_axis.h"

void get_translation(pcl::PointCloud<pcl::Normal>::Ptr pointNormals_src, pcl::PointCloud<pcl::Normal>::Ptr pointNormals_tgt, float lim, std::vector<std::vector<float>> axis, int N_hist_axis, Eigen::Matrix4f* translation_transform);

#include "get_translation.inl"

#endif // GET_TRANSLATION

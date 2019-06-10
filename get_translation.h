#ifndef GET_TRANSLATION
#define GET_TRANSLATION

#include "get_proj_axis.h"
#include "get_hist_axis.h"
#include "get_corr_axis.h"
#include "save_vector.h"
#include "get_walls.h"
//#include "transform.h"
#include "get_lim_axis.h"
#include "save_axis.h"

void get_translation(pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src, pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt, float lim, std::vector<Eigen::Vector3f> axis, float bin_width, bool sat, Eigen::Matrix4f* translation_transform);

#include "get_translation.inl"

#endif // GET_TRANSLATION

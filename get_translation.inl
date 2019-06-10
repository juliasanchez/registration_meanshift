void get_translation(pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src, pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt, float lim, std::vector<Eigen::Vector3f> axis, float bin_width, bool sat, Eigen::Matrix4f* translation_transform)
{
    float div_fact=100;
    if(!sat)
        bin_width /= div_fact;

    /// compute local frame axis--------------------------------------------------------------------------------------------------------------

    std::vector<Eigen::Vector3f> local_frame (3);
    local_frame[0]=axis[0];
    local_frame[2]=axis[0].cross(axis[1]);
    local_frame[1]=local_frame[2].cross(axis[0]);

    for(int i=0; i<local_frame.size(); i++)
        local_frame[i] /= local_frame[i].norm();

//    save_axis(local_frame[0], "axis_local_x.csv");
//    save_axis(local_frame[1], "axis_local_y.csv");
//    save_axis(local_frame[2], "axis_local_z.csv");

    ///get histograms--------------------------------------------------------------------------------------------------------------
    std::vector<std::vector<float>> axis_lim(3,std::vector<float>(2,0.0));
    std::vector<int> N_hist(3);

    //filter clouds to keep walls on x--------------------------------------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
    get_walls(pointNormals_src, lim, axis[0], cloud_src_filtered1);
    get_walls(pointNormals_tgt, lim, axis[0], cloud_tgt_filtered1);

    if(cloud_src_filtered1->points.size()==0 || cloud_tgt_filtered1->points.size()==0)
    return;
    get_lim_axis(cloud_src_filtered1, cloud_tgt_filtered1, axis[0], axis_lim[0]);
    N_hist[0]=(int)(round((axis_lim[0][1]-axis_lim[0][0])/bin_width));

//    pcl::io::savePCDFileASCII ("filt_src1.csv", *cloud_src_filtered1);
//    pcl::io::savePCDFileASCII ("filt_tgt1.csv", *cloud_tgt_filtered1);

    //filter clouds to keep walls on y--------------------------------------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
    get_walls(pointNormals_src, lim, axis[1], cloud_src_filtered2);
    get_walls(pointNormals_tgt, lim, axis[1], cloud_tgt_filtered2);

    if(cloud_src_filtered2->points.size()==0 || cloud_tgt_filtered2->points.size()==0)
    return;
    get_lim_axis(cloud_src_filtered2, cloud_tgt_filtered2, axis[1], axis_lim[1]);
    N_hist[1]=(int)(round((axis_lim[1][1]-axis_lim[1][0])/bin_width));

//    pcl::io::savePCDFileASCII ("filt_src2.csv", *cloud_src_filtered2);
//    pcl::io::savePCDFileASCII ("filt_tgt2.csv", *cloud_tgt_filtered2);

    //filter clouds to keep ground and roof on z--------------------------------------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_filtered3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_filtered3(new pcl::PointCloud<pcl::PointXYZ>);
    get_walls(pointNormals_src, lim, axis[2], cloud_src_filtered3);
    get_walls(pointNormals_tgt, lim, axis[2], cloud_tgt_filtered3);

//    pcl::io::savePCDFileASCII ("filt_src3.csv", *cloud_src_filtered3);
//    pcl::io::savePCDFileASCII ("filt_tgt3.csv", *cloud_tgt_filtered3);
//    pcl::io::savePCDFileASCII ("before_filt_src.csv", *pointNormals_src);

    if(cloud_src_filtered3->points.size()==0 || cloud_tgt_filtered3->points.size()==0)
    return;
    get_lim_axis(cloud_src_filtered3, cloud_tgt_filtered3, axis[2], axis_lim[2]);
    N_hist[2]=(int)(round((axis_lim[2][1]-axis_lim[2][0])/bin_width));

    std::vector<float> hist1_axis1(N_hist[0], 0.0);
    std::vector<float> hist2_axis1(N_hist[0], 0.0);
    std::vector<float> hist1_axis2(N_hist[1], 0.0);
    std::vector<float> hist2_axis2(N_hist[1], 0.0);
    std::vector<float> hist1_axis3(N_hist[2], 0.0);
    std::vector<float> hist2_axis3(N_hist[2], 0.0);

    bool moy= !sat;
    get_hist_axis(axis_lim[0], axis[0], cloud_src_filtered1, moy, hist1_axis1);  // moy corresponds to the activation or not of the mean filter on the histogram (if the bins are very small)
    get_hist_axis(axis_lim[0], axis[0], cloud_tgt_filtered1, moy, hist2_axis1);

    get_hist_axis(axis_lim[1], axis[1], cloud_src_filtered2, moy, hist1_axis2);
    get_hist_axis(axis_lim[1], axis[1], cloud_tgt_filtered2, moy, hist2_axis2);

    get_hist_axis(axis_lim[2], axis[2], cloud_src_filtered3, moy, hist1_axis3);
    get_hist_axis(axis_lim[2], axis[2], cloud_tgt_filtered3, moy, hist2_axis3);

    ///save histograms--------------------------------------------------------------------------------------------------------------
//    save_vector (hist1_axis1, "hist1_axis1.csv");

    int translation_axis1;
    int translation_axis2;
    int translation_axis3;

    std::vector<float> corr_axis1(2*N_hist[0]-1, 0.0);
    std::vector<float> corr_axis2(2*N_hist[1]-1, 0.0);
    std::vector<float> corr_axis3(2*N_hist[2]-1, 0.0);

    ///compute corr function for axis1--------------------------------------------------------------------------------------------------------------

    int margin; // margin is the number of bins on which to compute correlation (depends on which overlap is expected)
    if (!sat)  //si on est dans la passe 1 (coarse translation) sat=1 si on est dans la passe 2 sat=0
        margin = (int)(3*div_fact/2);
    else
        margin = 0;

    get_corr_axis(hist1_axis1, hist2_axis1, margin, corr_axis1, &translation_axis1);
//    save_vector(corr_axis1, "corr_axis1.csv");

    float delta1=(float)(axis_lim[0][1]-axis_lim[0][0]) / (float)(N_hist[0]);
    float delta_axis1 = translation_axis1 * delta1;

    ///--------------------------------------------

    float x1 = delta_axis1*local_frame[0](0);
    float y1 = delta_axis1*local_frame[0](1);
    float z1 = delta_axis1*local_frame[0](2);

    ///compute corr function for axis2--------------------------------------------------------------------------------------------------------------

    get_corr_axis(hist1_axis2, hist2_axis2, margin, corr_axis2, &translation_axis2);

//    save_vector(corr_axis2, "corr_axis2.csv");

    float delta2 = (float)(axis_lim[1][1] - axis_lim[1][0]) / (float)(N_hist[1]);

    float dot = axis[0].dot(axis[1]);
    double alpha01 = acos(dot);

    double delta_m = translation_axis2 * delta2;
    double delta_axis2 = ( delta_m - delta_axis1*cos(alpha01) )/sin(alpha01);

    ///-------------------------------------------

    float x2=delta_axis2*local_frame[1](0);
    float y2=delta_axis2*local_frame[1](1);
    float z2=delta_axis2*local_frame[1](2);

    ///compute corr function for axis3--------------------------------------------------------------------------------------------------------------

    get_corr_axis(hist1_axis3, hist2_axis3, margin, corr_axis3, &translation_axis3);
//    save_vector(corr_axis2, "corr_axis2.csv");

    float delta3 = (float)(axis_lim[2][1] - axis_lim[2][0]) / (float)(N_hist[2]);

    dot = local_frame[1].dot(axis[2]);
    double alpha12 = acos(dot);
    dot = local_frame[0].dot(axis[2]);
    double alpha02 = acos(dot);

    double delta_n = translation_axis3 * delta3;
    double delta_axis3 = ( (delta_n - delta_axis1*cos(alpha02))/sin(alpha02) - delta_axis2*cos(alpha12) ) / sin(alpha12);

    ///-------------------------------------------

    float x3=delta_axis3*local_frame[2](0);
    float y3=delta_axis3*local_frame[2](1);
    float z3=delta_axis3*local_frame[2](2);

    dot = axis[2].dot(local_frame[2]);
    if(dot<0)
    {
        x3=-x3;
        y3=-y3;
        z3=-z3;
    }

    ///compute total transformation matrix--------------------------------------------------------------------------------------------------------------

    (*translation_transform)(0,3)=x1+x2+x3;
    (*translation_transform)(1,3)=y1+y2+y3;
    (*translation_transform)(2,3)=z1+z2+z3;

}

void get_translation(pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src, pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt, float lim, std::vector<Eigen::Vector3f> axis, float bin_width, bool sat, Eigen::Matrix4f* translation_transform)
{
    float div_fact=100;
    if(!sat)
        bin_width /= div_fact;

    bool moy= !sat;

    int margin = 0; // margin is the number of bins on which to compute correlation (depends on which overlap is expected)
    if (!sat)  //si on est dans la passe 1 (coarse translation) sat=1 si on est dans la passe 2 sat=0
        margin = (int)(1.5*div_fact);
    else
        margin = 0;

    std::vector<std::vector<float>> axis_lim(3,std::vector<float>(2,0.0));
    std::vector<int> N_hist(3);

    double delta_axis1 = 0;
    double delta_axis2 = 0;
    double delta_axis3 = 0;

    int translation_axis1 = 0;
    int translation_axis2 = 0;
    int translation_axis3 = 0;

    std::vector<float> corr_axis1;
    std::vector<float> corr_axis2;
    std::vector<float> corr_axis3;

    Eigen::Vector3f trans1;
    Eigen::Vector3f trans2;
    Eigen::Vector3f trans3;

    /// compute local frame axis--------------------------------------------------------------------------------------------------------------

    std::vector<Eigen::Vector3f> local_frame (3);
    local_frame[0]=axis[0];
    local_frame[2]=axis[0].cross(axis[1]);
    local_frame[1]=local_frame[2].cross(axis[0]);

    for(int i=0; i<local_frame.size(); i++)
        local_frame[i] /= local_frame[i].norm();

    std::set<double> proj_src;
    std::set<double> proj_tgt;

    ///get histograms--------------------------------------------------------------------------------------------------------------

    //filter clouds to keep walls on x--------------------------------------------------------------------------------------------------------------

    proj_src.clear();
    proj_tgt.clear();

    get_walls(pointNormals_src, lim, axis[0], proj_src);
    get_walls(pointNormals_tgt, lim, axis[0], proj_tgt);

    if(proj_src.size()==0 || proj_tgt.size()==0)
        std::cout<<"lack of direction : no moving on first axis"<<std::endl;
    else
    {
        get_lim_axis(proj_src, proj_tgt, axis_lim[0]);
        N_hist[0]=(int)(round((axis_lim[0][1]-axis_lim[0][0])/bin_width))+1;
        corr_axis1.resize(2*N_hist[0]-1, 0.0);

        std::vector<float> hist1_axis1(N_hist[0], 0.0);
        std::vector<float> hist2_axis1(N_hist[0], 0.0);
        get_hist_axis(axis_lim[0], proj_src, moy, hist1_axis1);  // moy corresponds to the activation or not of the mean filter on the histogram (if the bins are very small)
        get_hist_axis(axis_lim[0], proj_tgt, moy, hist2_axis1);

        //compute corr function for axis1--------------------------------------------------------------------------------------------------------------

        get_corr_axis(hist1_axis1, hist2_axis1, margin, corr_axis1, &translation_axis1);
        float delta1=(float)(axis_lim[0][1]-axis_lim[0][0]) / (float)(N_hist[0]);
        delta_axis1 = translation_axis1 * delta1;
    }
    trans1 = delta_axis1*local_frame[0];

    //filter clouds to keep walls on y--------------------------------------------------------------------------------------------------------------
    proj_src.clear();
    proj_tgt.clear();

    get_walls(pointNormals_src, lim, axis[1], proj_src);
    get_walls(pointNormals_tgt, lim, axis[1], proj_tgt);

    if(proj_src.size()==0 || proj_tgt.size()==0)
        std::cout<<"lack of direction : no moving on second axis"<<std::endl;
    else
    {
        get_lim_axis(proj_src, proj_tgt, axis_lim[1]);
        N_hist[1]=(int)(round((axis_lim[1][1]-axis_lim[1][0])/bin_width))+1;

        corr_axis2.resize(2*N_hist[1]-1, 0.0);

        std::vector<float> hist1_axis2(N_hist[1], 0.0);
        std::vector<float> hist2_axis2(N_hist[1], 0.0);

        get_hist_axis(axis_lim[1], proj_src, moy, hist1_axis2);
        get_hist_axis(axis_lim[1], proj_tgt, moy, hist2_axis2);

        ///compute corr function for axis2--------------------------------------------------------------------------------------------------------------

        get_corr_axis(hist1_axis2, hist2_axis2, margin, corr_axis2, &translation_axis2);
        float delta2 = (float)(axis_lim[1][1] - axis_lim[1][0]) / (float)(N_hist[1]);
        double dot = axis[0].dot(axis[1]);
        double alpha01 = acos(dot);
        double delta_m = translation_axis2 * delta2;
        delta_axis2 = ( delta_m - delta_axis1*cos(alpha01) )/sin(alpha01);
    }
    trans2 = delta_axis2 * local_frame[1];

    //filter clouds to keep ground and roof on z--------------------------------------------------------------------------------------------------------------

    proj_src.clear();
    proj_tgt.clear();

    get_walls(pointNormals_src, lim, axis[2], proj_src);
    get_walls(pointNormals_tgt, lim, axis[2], proj_tgt);

//    pcl::io::savePCDFileASCII ("filt_src3.csv", *cloud_src_filtered);
//    pcl::io::savePCDFileASCII ("filt_tgt3.csv", *cloud_tgt_filtered);
//    pcl::io::savePCDFileASCII ("before_filt_src.csv", *pointNormals_src);

    if(proj_src.size()==0 || proj_tgt.size()==0)
        std::cout<<"lack of direction : no moving on third axis"<<std::endl;
    else
    {
        get_lim_axis(proj_src, proj_tgt, axis_lim[2]);
        N_hist[2]=(int)(round((axis_lim[2][1]-axis_lim[2][0])/bin_width))+1;

        corr_axis3.resize(2*N_hist[2]-1, 0.0);

        std::vector<float> hist1_axis3(N_hist[2], 0.0);
        std::vector<float> hist2_axis3(N_hist[2], 0.0);
        get_hist_axis(axis_lim[2], proj_src, moy, hist1_axis3);
        get_hist_axis(axis_lim[2], proj_tgt, moy, hist2_axis3);

        ///compute corr function for axis3--------------------------------------------------------------------------------------------------------------

        get_corr_axis(hist1_axis3, hist2_axis3, margin, corr_axis3, &translation_axis3);
    //    save_vector(corr_axis2, "corr_axis2.csv");
        float delta3 = (float)(axis_lim[2][1] - axis_lim[2][0]) / (float)(N_hist[2]);
        double dot = local_frame[1].dot(axis[2]);
        double alpha12 = acos(dot);
        dot = local_frame[0].dot(axis[2]);
        double alpha02 = acos(dot);
        double delta_n = translation_axis3 * delta3;
        delta_axis3 = ( (delta_n - delta_axis1*cos(alpha02))/sin(alpha02) - delta_axis2*cos(alpha12) ) / sin(alpha12);
    }

    trans3 = delta_axis3*local_frame[2];

    if(axis[2].dot(local_frame[2])<0)
        trans3 *= -1;

    ///save histograms--------------------------------------------------------------------------------------------------------------
//    save_vector (hist1_axis1, "hist1_axis1.csv");
//    save_vector (hist1_axis2, "hist1_axis2.csv");
//    save_vector (hist1_axis3, "hist1_axis3.csv");

//    save_vector (hist2_axis1, "hist2_axis1.csv");
//    save_vector (hist2_axis2, "hist2_axis2.csv");
//    save_vector (hist2_axis3, "hist2_axis3.csv");


    ///compute total transformation matrix--------------------------------------------------------------------------------------------------------------

    Eigen::Vector3f final_trans = trans1 + trans2 + trans3;
    (*translation_transform)(0,3) = final_trans(0);
    (*translation_transform)(1,3) = final_trans(1);
    (*translation_transform)(2,3) = final_trans(2);

}

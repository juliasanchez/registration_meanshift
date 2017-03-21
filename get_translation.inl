void get_translation(pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src, pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt, float lim, std::vector<std::vector<float>> axis, int N_hist_axis, Eigen::Matrix4f* translation_transform)
{
    ///get histograms--------------------------------------------------------------------------------------------------------------

    std::vector<float> hist1_axis1(N_hist_axis, 0.0);
    std::vector<float> hist2_axis1(N_hist_axis, 0.0);
    std::vector<float> hist1_axis2(N_hist_axis, 0.0);
    std::vector<float> hist2_axis2(N_hist_axis, 0.0);
    std::vector<float> hist1_axis3(N_hist_axis, 0.0);
    std::vector<float> hist2_axis3(N_hist_axis, 0.0);
    std::vector<std::vector<float>> axis_lim(3,std::vector<float>(2,0.0));

    //filter clouds to keep walls on x--------------------------------------------------------------------------------------------------------------
    pcl::PointCloud<pcl_point>::Ptr cloud_src_filtered1(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt_filtered1(new pcl::PointCloud<pcl_point>);
    get_walls(pointNormals_src, lim, axis[0], cloud_src_filtered1);
    get_walls(pointNormals_tgt, lim, axis[0], cloud_tgt_filtered1);
    pcl::io::savePCDFileASCII ("filtered_src_x.pcd", *cloud_src_filtered1);
    pcl::io::savePCDFileASCII ("filtered_tgt_x.pcd", *cloud_tgt_filtered1);
    get_lim_axis(cloud_src_filtered1, cloud_tgt_filtered1, axis[0], axis_lim[0]);
    get_hist_axis(axis_lim[0], axis[0], cloud_src_filtered1, hist1_axis1);
    get_hist_axis(axis_lim[0], axis[0], cloud_tgt_filtered1, hist2_axis1);

    //filter clouds to keep walls on y--------------------------------------------------------------------------------------------------------------
    pcl::PointCloud<pcl_point>::Ptr cloud_src_filtered2(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt_filtered2(new pcl::PointCloud<pcl_point>);
    get_walls(pointNormals_src, lim, axis[1], cloud_src_filtered2);
    get_walls(pointNormals_tgt, lim, axis[1], cloud_tgt_filtered2);
    pcl::io::savePCDFileASCII ("filtered_src_y.pcd", *cloud_src_filtered2);
    pcl::io::savePCDFileASCII ("filtered_tgt_y.pcd", *cloud_tgt_filtered2);
    get_lim_axis(cloud_src_filtered2, cloud_tgt_filtered2, axis[1], axis_lim[1]);
    get_hist_axis(axis_lim[1], axis[1], cloud_src_filtered2, hist1_axis2);
    get_hist_axis(axis_lim[1], axis[1], cloud_tgt_filtered2, hist2_axis2);

    //filter clouds to keep ground and roof on z--------------------------------------------------------------------------------------------------------------
    pcl::PointCloud<pcl_point>::Ptr cloud_src_filtered3(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt_filtered3(new pcl::PointCloud<pcl_point>);
    get_walls(pointNormals_src, lim, axis[2], cloud_src_filtered3);
    get_walls(pointNormals_tgt, lim, axis[2], cloud_tgt_filtered3);
    get_lim_axis(cloud_src_filtered3, cloud_tgt_filtered3, axis[2], axis_lim[2]);
    get_hist_axis(axis_lim[2], axis[2], cloud_src_filtered3, hist1_axis3);
    get_hist_axis(axis_lim[2], axis[2], cloud_tgt_filtered3, hist2_axis3);


    norm_hist(hist1_axis1);
    norm_hist(hist1_axis2);
    norm_hist(hist2_axis1);
    norm_hist(hist2_axis2);
    norm_hist(hist1_axis3);
    norm_hist(hist2_axis3);

    ///save histograms--------------------------------------------------------------------------------------------------------------

    save_vector (hist1_axis1, "hist1_axis1.csv");
    save_vector (hist1_axis2, "hist1_axis2.csv");
    save_vector (hist2_axis1, "hist2_axis1.csv");
    save_vector (hist2_axis2, "hist2_axis2.csv");
    save_vector (hist1_axis3, "hist1_axis3.csv");
    save_vector (hist2_axis3, "hist2_axis3.csv");

    ///compute corr function for axis1--------------------------------------------------------------------------------------------------------------

    std::vector<float> corr_axis1(2*N_hist_axis-1, 0.0);
    std::vector<float> corr_axis2(2*N_hist_axis-1, 0.0);
    std::vector<float> corr_axis3(2*N_hist_axis-1, 0.0);

    int translation_axis1;
    int translation_axis2;
    int translation_axis3;

    get_corr_axis(hist1_axis1, hist2_axis1, corr_axis1, &translation_axis1);
    save_vector(corr_axis1, "corr_axis1.csv");

    float delta1=(float)(axis_lim[0][1]-axis_lim[0][0]) / (float)(N_hist_axis);
    float delta_axis1=(translation_axis1-N_hist_axis+1)*delta1;

    float x1=delta_axis1*axis[0][0];
    float y1=delta_axis1*axis[0][1];
    float z1=delta_axis1*axis[0][2];

//    std::cout<<"movement for axis 1: "<<std::endl;
//    std::cout<<std::endl<<"  x translation: "<<x1<<"  y translation: "<<y1<<"  z translation: "<<z1<<std::endl<<std::endl;


    ///compute corr function for axis2--------------------------------------------------------------------------------------------------------------

    get_corr_axis(hist1_axis2, hist2_axis2, corr_axis2, &translation_axis2);
    float delta2=(float)(  (axis_lim[1][1]-axis_lim[1][0])/(float)(N_hist_axis)  );

    save_vector(corr_axis2, "corr_axis2.csv");

    std::vector<float> axis_y (3);
    axis_y[0]=axis[2][1]*axis[0][2]-axis[2][2]*axis[0][1];
    axis_y[1]=axis[2][2]*axis[0][0]-axis[2][0]*axis[0][2];
    axis_y[2]=axis[2][0]*axis[0][1]-axis[2][1]*axis[0][0];
    save_axis(axis_y, "axis_y1.csv");

    float dot = axis[0][0]*axis[1][0]+axis[0][1]*axis[1][1]+axis[0][2]*axis[1][2];
    std::vector<float> cross12 = {axis[0][1]*axis[1][2]-axis[0][2]*axis[1][1], axis[0][2]*axis[1][0]-axis[0][0]*axis[1][2], axis[0][0]*axis[1][1]-axis[0][1]*axis[1][0]};
    std::vector<float> cross11 = {axis[0][1]*axis_y[2]-axis[0][2]*axis_y[0], axis[0][2]*axis_y[0]-axis[0][0]*axis_y[2], axis[0][0]*axis_y[1]-axis[0][1]*axis_y[0]};
    float dot_cross= cross12[0]*cross11[0]+cross12[1]*cross11[1]+cross12[2]*cross11[2];
    double alpha = atan(dot_cross/abs(dot));
    double delta_m = (translation_axis2 - N_hist_axis + 1) * delta2;
    double delta_axis2 = (-delta_axis1*cos(alpha) + delta_m)/sin(alpha);
    if(abs(alpha)>M_PI/2)
    {
        delta_axis2 = (-delta_axis1*cos(alpha) - delta_m)/sin(alpha);
    }

    float x2=delta_axis2*axis_y[0];
    float y2=delta_axis2*axis_y[1];
    float z2=delta_axis2*axis_y[2];

//    std::cout<<"movement for axis 2: "<<std::endl;
//    std::cout<<std::endl<<"  x translation: "<<x2<<"  y translation: "<<y2<<"  z translation: "<<z2<<std::endl<<std::endl;

    ///compute corr function for axis3--------------------------------------------------------------------------------------------------------------

    get_corr_axis(hist1_axis3, hist2_axis3, corr_axis3, &translation_axis3);
    save_vector(corr_axis3, "corr_axis3.csv");

    float delta3=(float)(  (axis_lim[2][1]-axis_lim[2][0])/(float)(N_hist_axis)  );

    float x3=(translation_axis3-N_hist_axis+1)*delta3*axis[2][0];
    float y3=(translation_axis3-N_hist_axis+1)*delta3*axis[2][1];
    float z3=(translation_axis3-N_hist_axis+1)*delta3*axis[2][2];

//    std::cout<<"movement for axis 3: "<<std::endl;
//    std::cout<<std::endl<<"  x translation: "<<x3<<"  y translation: "<<y3<<"  z translation: "<<z3<<std::endl<<std::endl;

    ///compute total transformation matrix--------------------------------------------------------------------------------------------------------------

    Eigen::Matrix4f translation_transform0 = Eigen::Matrix4f::Zero();
    translation_transform0(0,3)=x1+x2+x3;
    translation_transform0(1,3)=y1+y2+y3;
    translation_transform0(2,3)=z1+z2+z3;

    *translation_transform=translation_transform0;

}

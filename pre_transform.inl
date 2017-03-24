void pre_transform(float theta_init, std::vector<float> rot_ax, Eigen::Matrix4f* transform_init )
{
    Eigen::Matrix4f transform_init0 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_phi_init = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_theta_init = Eigen::Matrix4f::Identity();

    transform_theta_init  (0,0) = rot_ax[0]*rot_ax[0]*(1-cos (theta_init))+cos (theta_init);
    transform_theta_init  (0,1) = rot_ax[0]*rot_ax[1]*(1-cos (theta_init))-rot_ax[2]*sin (theta_init);
    transform_theta_init  (0,2) = rot_ax[0]*rot_ax[2]*(1-cos (theta_init))+rot_ax[1]*sin (theta_init);
    transform_theta_init  (1,0) = rot_ax[1]*rot_ax[0]*(1-cos (theta_init))+rot_ax[2]*sin (theta_init);
    transform_theta_init  (1,1) = rot_ax[1]*rot_ax[1]*(1-cos (theta_init))+cos (theta_init);
    transform_theta_init  (1,2) = rot_ax[1]*rot_ax[2]*(1-cos (theta_init))-rot_ax[0]*sin (theta_init);
    transform_theta_init  (2,0) = rot_ax[2]*rot_ax[0]*(1-cos (theta_init))-rot_ax[1]*sin (theta_init);
    transform_theta_init  (2,1) = rot_ax[2]*rot_ax[1]*(1-cos (theta_init))+rot_ax[0]*sin (theta_init);
    transform_theta_init  (2,2) = rot_ax[2]*rot_ax[2]*(1-cos (theta_init))+cos (theta_init);


    transform_init0=transform_phi_init*transform_theta_init;
    std::cout << "initial transform : "<<std::endl<<transform_init0 << std::endl;

    *transform_init=transform_init0;
}

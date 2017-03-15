void transform(float phi, float theta, Eigen::Matrix4f* transform)
{

    Eigen::Matrix4f transform_phi = Eigen::Matrix4f::Identity();
    transform_phi (0,0) = cos (phi);
    transform_phi (0,1) = -sin(phi);
    transform_phi (1,0) = sin (phi);
    transform_phi (1,1) = cos (phi);

    Eigen::Matrix4f transform_theta = Eigen::Matrix4f::Identity();
    transform_theta (0,0) = cos (theta);
    transform_theta (0,2) = sin(theta);
    transform_theta (2,0) = -sin (theta);
    transform_theta (2,2) = cos (theta);

    *transform=transform_theta*transform_phi;

}

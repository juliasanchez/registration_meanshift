void pre_process(std::string pcd_file,float sample, float normal_radius, float far, cloud *cloud_manager, double* reso)
{
    cloud_manager->load(pcd_file);

    std::cout << "initial points number : " << cloud_manager->getSize () << std::endl;
    cloud_manager->sample(0.005);
    cloud_manager->clean(far);

    if(abs(sqrt(pow(cloud_manager->getInputCloud()->points[0].normal_x,2) + pow(cloud_manager->getInputCloud()->points[0].normal_y,2) + pow(cloud_manager->getInputCloud()->points[0].normal_z,2))-1)>1e-3)
        cloud_manager->getNormals(normal_radius);
    cloud_manager->orient();

    cloud_manager->sample(sample);

    *reso=cloud_manager->computeCloudResolution ();
    std::cout << "cloud resolution after preprocess: "<<*reso<<std::endl<<std::endl;
}

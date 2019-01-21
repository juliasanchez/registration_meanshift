void pre_process(std::string pcd_file,float sample, float normal_radius, float far, cloud<pcl_point> *cloud_in, double* reso)
{
    cloud_in->load(pcd_file);

    std::cout << " input cloud number of points : " << cloud_in->getSize () << std::endl;
    cloud_in->sample(0.005);

    std::cout << " after first sampling 5mm : " << cloud_in->getSize () << std::endl;
    cloud_in->clean(far);
    std::cout << " after cleaning : " << cloud_in->getSize () << std::endl;
    cloud_in->getNormals(normal_radius);
    cloud_in->orient();

    cloud_in->sample(sample);

    std::cout << " after sampling : " << cloud_in->getSize () << std::endl<<std::endl;
    *reso=cloud_in->computeCloudResolution ();
    std::cout << "cloud resolution : "<<*reso<<std::endl<<std::endl;
}

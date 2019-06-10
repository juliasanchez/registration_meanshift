void save_cluster(Eigen::Vector3f& vec, std::string file_name)
{
    std::ofstream file (file_name);
    for(int q = 1; q < 1500; q ++)
    {
	    for(int k = 0; k < 3; k ++)
	    {
                file <<vec(k)*q*0.001 << ", " ;
	    }
	    file <<"\n";
    }
    file.close();
}

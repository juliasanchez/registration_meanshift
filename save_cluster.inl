void save_cluster(std::vector<double>& vec, std::string file_name)
{
 std::ofstream file (file_name);
    for(int q = 1; q < 20; q ++)
    {
	    for(int k = 0; k < 3; k ++)
	    {
		file <<vec[k]*q << ", " ;
	    }
	    file <<"\n";
    }
    file.close();
}

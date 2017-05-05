void save_axis(std::vector<float>& vec, std::string file_name)
{
    	
    std::ofstream file (file_name);
    for(int q = 1; q < 800; q ++)
    {
	    for(int k = 0; k < 3; k ++)
	    {
                file <<vec[k]*q*0.01 << ", " ;
	    }
	    file <<"\n";
    }
    file.close();
}

void save_axis(std::vector<float>& vec, std::string file_name)
{
    	
    ofstream file (file_name);
    for(int q = 1; q < 4; q ++)
    {
	    for(int k = 0; k < 3; k ++)
	    {
		file <<vec[k]*q << ", " ;
	    }
	    file <<"\n";
    }
    file.close();
}

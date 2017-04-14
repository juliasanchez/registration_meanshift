void save_vector(std::vector<float>& vec, std::string file_name)
{
    std::ofstream file (file_name);
    for(int k = 0; k < vec.size(); k ++)
    {
        file <<vec[k] << "\n" ;
    }
    file.close();
}

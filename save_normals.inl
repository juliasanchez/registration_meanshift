void save_normals(std::vector<std::vector<double>>& mat, std::string file_name)
{
    std::ofstream file (file_name);

    for(int k = 0; k < mat.size(); k ++)
    {
        for(int l = 0; l < mat[0].size(); l ++)
        {
            if(l<2)
            file <<mat[k][l] << "," ;
            else
            file <<mat[k][l]<< "\n";
        }
    }


    file.close();
}

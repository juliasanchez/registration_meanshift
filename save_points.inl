void save_points(Eigen::MatrixXf& mat, std::string file_name)
{
    std::ofstream file (file_name);
    for(int k = 0; k < mat.rows(); k ++)
        file <<mat(k,0)<<","<<mat(k,1)<<","<<mat(k,2)<< "\n" ;
    file.close();
}

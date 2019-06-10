void density_filter(pcl::PointCloud<points>::Ptr cloud_in, float radius, int keep, std::vector<Eigen::Vector3f>& modes)
{
    pcl::KdTreeFLANN<points> tree;
    tree.setInputCloud(cloud_in);
    float large_radius = 0.3;

    pcl::PointCloud<points> cloud_out;

    std::vector<float> density(cloud_in->size());

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    for (int i=0; i< cloud_in->size(); i++)
    {
        density[i] = 0;
        if ( tree.radiusSearch (cloud_in->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
//           density[i]= pointIdxRadiusSearch.size();  //key = index, value =density
            for (int k=1; k<pointIdxRadiusSearch.size(); k++)
                density[i] += exp(-pointRadiusSquaredDistance[k]/(pow(radius/2,2)));
        }
    }

    std::vector<float> density_cpy = density;
    //select the 8 points with max density and their neighborhood in radius. + their neighborhood in "large_radius" is removed from the initial map to pick up the new max area at each iteration.

    int nbr_clusters = 8;
    std::vector<std::vector<int>> clus(nbr_clusters);
    std::vector<std::vector<int>> large_clus(nbr_clusters);
    for (int i=0; i<nbr_clusters; i++)
    {
        auto pr = std::max_element (density.begin(), density.end());

        tree.radiusSearch (cloud_in->points[  std::distance(density.begin(), pr)  ], large_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        if(pointIdxRadiusSearch.size()>10)
        {
            for (int k=0; k<pointIdxRadiusSearch.size(); k++)
            {
                large_clus[i].push_back(pointIdxRadiusSearch[k]);
                if(sqrt(pointRadiusSquaredDistance[k])<radius)
                    clus[i].push_back(pointIdxRadiusSearch[k]);
                density[pointIdxRadiusSearch[k]]=0;
            }
        }
        else
            break;
    }

    //remove too large clusters (curved walls)-----------------------------------------------------------------------------------------------------
    std::vector<Eigen::MatrixXf> points_of_clusters(nbr_clusters);

    for (int i=0; i<large_clus.size(); i++)
    {
        points_of_clusters[i] = Eigen::MatrixXf::Zero(large_clus[i].size(), 3);
        for(int j=0; j<large_clus[i].size(); j++)
            points_of_clusters[i].row(j) = Eigen::Vector3f (cloud_in->points[large_clus[i][j]].x, cloud_in->points[large_clus[i][j]].y, cloud_in->points[large_clus[i][j]].z );

        Eigen::MatrixXf centered_points (large_clus[i].size(), 3);
        Eigen::Vector3f mean_point = points_of_clusters[i].colwise().mean();
        centered_points = points_of_clusters[i] - Eigen::VectorXf::Ones(large_clus[i].size())*(mean_point.transpose());

        Eigen::Matrix3f C = centered_points.transpose()*centered_points/large_clus[i].size();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(C);
        float lambda_max = es.eigenvalues().maxCoeff();
        float alpha_threshold = 15 * M_PI/180;
        if(2*sqrt(lambda_max) > alpha_threshold)
        {
            std::cout<<"curved wall detected"<<std::endl<<std::endl;
            clus.erase(clus.begin() + i);
            large_clus.erase(large_clus.begin() + i);
            --i;
        }
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------------

    modes.resize(clus.size());

    for (int i=0; i<clus.size(); i++)
    {
        for(int j=0; j<clus[i].size(); j++)
        {
            if(((double) rand() / (RAND_MAX))<(float)(keep)/(float)(clus[i].size()))
            {
                points point;
                point.x = cloud_in->points[clus[i][j]].x;
                point.y = cloud_in->points[clus[i][j]].y;
                point.z = cloud_in->points[clus[i][j]].z;
                cloud_out.points.push_back(point);
            }

            modes[i](0) += density_cpy[clus[i][j]]*cloud_in->points[clus[i][j]].x;
            modes[i](1) += density_cpy[clus[i][j]]*cloud_in->points[clus[i][j]].y;
            modes[i](2) += density_cpy[clus[i][j]]*cloud_in->points[clus[i][j]].z;
        }
    }

    for (int i=0; i<modes.size(); ++i)
        modes[i] /= modes[i].norm();

    cloud_out.width = (uint32_t)  cloud_out.points.size();
    cloud_out.height = 1;
    *cloud_in=cloud_out;
}

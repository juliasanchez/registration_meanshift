void density_filter(pcl::PointCloud<points>::Ptr cloud_in, float angle_uncertainty_small, float angle_uncertainty_large, int keep)
{
    std::cout<<std::endl<<std::endl;
    pcl::KdTreeFLANN<points> tree;
    tree.setInputCloud(cloud_in);

    int N_points_min = 1; // number of points minimum to be considered as cluster

    float radius = sqrt(2*(1-cos(angle_uncertainty_small)));
    float large_radius = sqrt(2*(1-cos(angle_uncertainty_large)));

    std::vector<float> density(cloud_in->size());

    #pragma omp parallel for schedule(dynamic) num_threads(omp_get_max_threads()) shared(tree, cloud_in, angle_uncertainty_small, density)
    for (int i=0; i< cloud_in->size(); i++)
    {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        density[i] = 0;
//        float norm = sqrt(pow(cloud_in->points[i].x,2) + pow(cloud_in->points[i].y,2) + pow(cloud_in->points[i].z,2));
//        float radius = norm * sqrt(2*(1-cos(angle_uncertainty_small)));
        if ( tree.radiusSearch (cloud_in->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            for (int k=1; k<pointIdxRadiusSearch.size(); k++)
                density[i] += exp(-pointRadiusSquaredDistance[k]/(pow(radius/2,2)));
        }
    }

    //select the 8 points with max density and their neighborhood in radius. + their neighborhood in "large_radius" is removed from the initial map to pick up the new max area at each iteration.

    int nbr_clusters = 8;
    std::vector<std::vector<int>> clus(nbr_clusters);
    std::vector<std::vector<int>> large_clus(nbr_clusters);
    for (int i=0; i<nbr_clusters; i++)
    {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        auto pr = std::max_element (density.begin(), density.end());

        pcl::PointXYZ pt = cloud_in->points[  std::distance(density.begin(), pr)  ];
//        float norm = sqrt(pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2));
//        float large_radius = norm * sqrt(2*(1-cos(angle_uncertainty_large)));
//        float radius = norm * sqrt(2*(1-cos(angle_uncertainty_small)));
        tree.radiusSearch (pt, large_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        if(pointIdxRadiusSearch.size()>N_points_min)
        {
            for (int k=0; k<pointIdxRadiusSearch.size(); k++)
            {
                large_clus[i].push_back(pointIdxRadiusSearch[k]);
                if(sqrt(pointRadiusSquaredDistance[k])<radius)
                    clus[i].push_back(pointIdxRadiusSearch[k]);
                density[pointIdxRadiusSearch[k]] = 0;             //will not be seed
            }
        }
        else
        {
            std::cout<<"Not added : not enough points : "<<pointIdxRadiusSearch.size()<<std::endl;
            break;
        }
    }


    //remove too small clusters (not walls)-----------------------------------------------------------------------------------------------------

    for (int i=0; i<clus.size(); i++)
    {
        if(clus[i].size() < N_points_min)
        {
            std::cout<<"clus n° "<< i<< "removed : not enough points : "<<clus[i].size()<<std::endl;
            clus.erase(clus.begin() + i);
            large_clus.erase(large_clus.begin() + i);
            --i;
        }
    }

    //remove too large clusters (curved walls)-----------------------------------------------------------------------------------------------------
    std::vector<Eigen::MatrixXf> points_of_clusters(nbr_clusters);

    for (int i=0; i<large_clus.size(); i++)
    {
        points_of_clusters[i] = Eigen::MatrixXf::Zero(large_clus[i].size(), 3);
        for(int j=0; j<large_clus[i].size(); j++)
            points_of_clusters[i].row(j) = Eigen::Vector3f (cloud_in->points[large_clus[i][j]].x, cloud_in->points[large_clus[i][j]].y, cloud_in->points[large_clus[i][j]].z );

        Eigen::MatrixXf centered_points (large_clus[i].size(), 3);
        Eigen::Vector3f mean_point = Eigen::Vector3f (cloud_in->points[clus[i][0]].x, cloud_in->points[clus[i][0]].y, cloud_in->points[clus[i][0]].z ); //points_of_clusters[i].colwise().mean();
        centered_points = points_of_clusters[i] - Eigen::VectorXf::Ones(large_clus[i].size())*(mean_point.transpose());

        Eigen::Matrix3f C = centered_points.transpose()*centered_points/centered_points.rows();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
        es.compute(C);
        float ratio = 1.0;
        if(es.eigenvalues()(1) != 0)
            ratio = es.eigenvalues()(2)/es.eigenvalues()(1);
        std::cout<<"ratio : "<<ratio<<std::endl;

        //--------------------------
//        int idx = rand();
        float lambda_max = es.eigenvalues()(2);
//        std::cout<<"Maximum standard deviation "<<idx<<" : "<<sqrt(lambda_max)<<std::endl;
//        std::stringstream stm;
//        stm<<"large_clus_"<<i<<"_"<<idx<<".csv";
//        std::cout<<"ratio "<<idx<<" : "<<ratio<<std::endl;
//        save_points(points_of_clusters[i], stm.str());
        float alpha_threshold = 10*M_PI/180;
//        if( 2*sqrt(lambda_max) > sqrt(2*(1-cos(alpha_threshold))) )
        //--------------------------

        if(ratio>3.0 && 2*sqrt(lambda_max) > sqrt(2*(1-cos(alpha_threshold))))
        {
            std::cout<<"curved wall detected"<<std::endl;
            clus.erase(clus.begin() + i);
            large_clus.erase(large_clus.begin() + i);
            --i;
        }
    }

    std::cout<<"number of clus detected by density:"<<clus.size()<<std::endl;

    //-------------------------------------------------------------------------------------------------------------------------------------------------
    pcl::PointCloud<points> cloud_out;

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
        }
    }

    cloud_out.width = cloud_out.points.size();
    cloud_out.height = 1;
    *cloud_in=cloud_out;
}

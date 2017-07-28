void density_filter(pcl::PointCloud<pcl_point>::Ptr cloud_in, float radius, int keep)
{
//    pcl::KdTreeFLANN<pcl_point> tree;
//    tree.setInputCloud(cloud_in);

//    pcl::PointCloud<pcl_point> cloud_out;
//    std::vector<int> pointIdxNKNSearch(points+1);
//    std::vector<float> pointNKNSquaredDistance(points+1);

//    for (int i=0; i< cloud_in->size(); i++)
//    {
//        if ( tree.nearestKSearch (cloud_in->points[i], points+1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//        {
//            if(sqrt(pointNKNSquaredDistance[points])<thresh)
//            {
//                pcl_point point;
//                point.x=cloud_in->points[i].x;
//                point.y=cloud_in->points[i].y;
//                point.z=cloud_in->points[i].z;
//                cloud_out.points.push_back(point);
//            }
//        }
//    }

//    cloud_out.width = (uint32_t)  cloud_out.points.size();
//    cloud_out.height = 1;
//    *cloud_in=cloud_out;

    pcl::KdTreeFLANN<pcl_point> tree;
    tree.setInputCloud(cloud_in);

    pcl::PointCloud<pcl_point> cloud_out;

    std::map<int, int> index_density;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    for (int i=0; i< cloud_in->size(); i++)
    {
        if ( tree.radiusSearch (cloud_in->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
         {
           index_density[i]= pointIdxRadiusSearch.size();  //key = index, value =density
         }
    }

    //items is ordered by key (index)

    std::vector<std::vector<int>> clus(6);
    for (int i=0; i<6; i++)
    {

        auto pr = std::max_element (index_density.begin(), index_density.end(), [] (std::pair<int, int> p1, std::pair<int,int> p2) {return p1.second < p2.second;} );

        tree.radiusSearch (cloud_in->points[  pr->first  ], 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        for (int k=0; k<pointIdxRadiusSearch.size(); k++)
        {
            if (index_density[pointIdxRadiusSearch[k]]!=0)
            {
                if( sqrt(pointRadiusSquaredDistance[k])<radius )
                {
                    clus[i].push_back(pointIdxRadiusSearch[k]);
                }
                index_density[pointIdxRadiusSearch[k]]=0;
            }
        }
    }

    for (int i=0; i<6; i++)
    {
        for(int j=0; j<clus[i].size(); j++)
        {
            if(keep>clus[i].size() || ((double) rand() / (RAND_MAX))<(float)(keep)/(float)(clus[i].size()))
            {
                pcl_point point;
                point.x=cloud_in->points[clus[i][j]].x;
                point.y=cloud_in->points[clus[i][j]].y;
                point.z=cloud_in->points[clus[i][j]].z;
                cloud_out.points.push_back(point);
            }
        }
    }

    cloud_out.width = (uint32_t)  cloud_out.points.size();
    cloud_out.height = 1;
    *cloud_in=cloud_out;
}

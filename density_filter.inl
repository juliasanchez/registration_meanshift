void density_filter(pcl::PointCloud<points>::Ptr cloud_in, float radius, int keep, std::vector<std::vector<float>>& modes)
{
    pcl::KdTreeFLANN<points> tree;
    tree.setInputCloud(cloud_in);

    pcl::PointCloud<points> cloud_out;

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

    //select the 6 points with max density and their neighborhood in radius. + their neighborhood in 0.3 is removed from the initial map to pick up the new max area at each iteration.

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

    int n=0;
    modes.resize(6);

    for (int i=0; i<6; i++)
    {
        n=0;
        modes[i].resize(3);
        for(int j=0; j<clus[i].size(); j++)
        {
            if(keep>clus[i].size() || ((double) rand() / (RAND_MAX))<(float)(keep)/(float)(clus[i].size()))
            {
                points point;
                point.x=cloud_in->points[clus[i][j]].x;
                point.y=cloud_in->points[clus[i][j]].y;
                point.z=cloud_in->points[clus[i][j]].z;
                cloud_out.points.push_back(point);
                modes[i][0] += point.x;
                modes[i][1] += point.y;
                modes[i][2] += point.z;
                ++n;
            }
        }
        modes[i][0] /= n;
        modes[i][1] /= n;
        modes[i][2] /= n;
    }

    for (int i=0; i<modes.size(); ++i)
    {
        float modu = sqrt(modes[i][0]*modes[i][0] + modes[i][1]*modes[i][1] + modes[i][2]*modes[i][2]);
        modes[i][0] /= modu;
        modes[i][1] /= modu;
        modes[i][2] /= modu;
    }

    cloud_out.width = (uint32_t)  cloud_out.points.size();
    cloud_out.height = 1;
    *cloud_in=cloud_out;
}

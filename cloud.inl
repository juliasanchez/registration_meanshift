cloud::cloud()
{
}

void cloud::getScale(float* Volume)
{
    pcl::PointNormal minPt, maxPt;
    pcl::getMinMax3D (*cloud_in, minPt, maxPt);
    float dist_x=maxPt.x-minPt.x;
    float dist_y=maxPt.y-minPt.y;
    float dist_z=maxPt.z-minPt.z;
    Eigen::Matrix4f scaling= Eigen::Matrix4f::Identity();
    *Volume= dist_x*dist_y*dist_z;
}

void cloud::setInputCloud(typename pcl::PointCloud<pcl::PointNormal>::Ptr cloudin)
{
	cloud_in=cloudin;
}

typename pcl::PointCloud<pcl::PointNormal>::Ptr cloud::getInputCloud()
{
    return cloud_in;
}

void cloud::getNormals(float radius)
{
    if(cloud_in->points[0].normal_x==0)
    {
        pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> normal_estimation;
        setTree();
        normal_estimation.setSearchMethod(boost::make_shared<pcl::search::KdTree<pcl::PointNormal>> (tree));
        normal_estimation.setRadiusSearch(radius);
        normal_estimation.setViewPoint (0, 0, 0);
	normal_estimation.setInputCloud(cloud_in);
        normal_estimation.compute(*cloud_in);
    }
}

void cloud::orient() const
{
    for (int i = 0; i<cloud_in->points.size(); ++i)
    {
        if( cloud_in->points[i].normal_x* cloud_in->points[i].x +  cloud_in->points[i].normal_y* cloud_in->points[i].y +  cloud_in->points[i].normal_z* cloud_in->points[i].z < 0 )
        {
            cloud_in->points[i].normal_x *= -1;
            cloud_in->points[i].normal_y *= -1;
            cloud_in->points[i].normal_z *= -1;
        }
    }
}

void cloud::setTree()
{
	tree.setInputCloud(cloud_in);
}

void cloud::sample(float samp)
{
    pcl::octree::OctreePointCloud<pcl::PointNormal> octree (samp);
    octree.setInputCloud (cloud_in);
    octree.defineBoundingBox ();
    octree.addPointsFromInputCloud ();

    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > *pointGrid = new std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> >;
    octree.getOccupiedVoxelCenters (*pointGrid);

    pcl::PointCloud<pcl::PointNormal> cloud_out;
//    cloud_out.resize(pointGrid->size());

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    setTree();
    for(int k=0; k<pointGrid->size(); ++k)
    {
        tree.nearestKSearch (pointGrid->at(k), 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        if(sqrt(pointNKNSquaredDistance[0])<sqrt(3)*samp/2)
            cloud_out.points.push_back(cloud_in->points[pointIdxNKNSearch[0]]);
//        cloud_out.points[k] = pointGrid->at(k);
    }

    cloud_out.is_dense=true;
    cloud_out.width=cloud_out.points.size();
    cloud_out.height=1;
    *cloud_in=cloud_out;
}

void cloud::rand_sample(float samp)
{
  pcl::RandomSample<pcl::PointNormal> sample;
  sample.setInputCloud (cloud_in);
  sample.setSample (samp);

  sample.filter(*cloud_in);
}

int cloud::getSize() const
{
       return cloud_in->size();
}

void cloud::load(std::string pcd_file)
{
    if( pcl::io::loadPCDFile<pcl::PointNormal>( pcd_file, *cloud_in ) == -1 )
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
}

void cloud::clean(float thresh)
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
    filter_far(cloud_in, thresh);
}

double cloud::computeCloudResolution ()
{
  double res = 0.0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);

  setTree();

  for (size_t i = 0; i < cloud_in->size (); ++i)
  {
        nres = tree.nearestKSearch (cloud_in->points[i], 2, indices, sqr_distances);
        res += sqrt (sqr_distances[1]);
  }
  res /= cloud_in->size ();
  return res;
}

void icp(pcl::PointCloud<pcl::PointNormal>::Ptr cloud1, pcl::PointCloud<pcl::PointNormal>::Ptr cloud2, float RANSAC_thresh, float rej_dist, int it, Eigen::Matrix4f *transformation)
{

        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
        boost::shared_ptr<pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointNormal> > rej (new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointNormal>() );
        rej->setInputSource(cloud1);
        rej->setInputTarget(cloud2);
        rej->setInlierThreshold(RANSAC_thresh);
        icp.addCorrespondenceRejector(rej);
        icp.setMaxCorrespondenceDistance(rej_dist);
        icp.setMaximumIterations((int)(it/2));
        icp.setTransformationEpsilon(0.0000001);

        icp.setInputSource(cloud1);
        icp.setInputTarget(cloud2);
        pcl::PointCloud<pcl::PointNormal>::Ptr output (new pcl::PointCloud<pcl::PointNormal>());
        icp.align(*output);
        Eigen::Matrix4f transfo1;
        transfo1=icp.getFinalTransformation();

        pcl::transformPointCloudWithNormals (*cloud1, *output, transfo1);
        icp.setMaxCorrespondenceDistance(rej_dist/2);
        icp.setInputSource(output);
        icp.setInputTarget(cloud2);
        pcl::PointCloud<pcl::PointNormal> output2;
        icp.align(output2);

        *transformation=icp.getFinalTransformation()*transfo1;

}

#include <iostream>
#include <string>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <pcl/registration/transformation_estimation_svd.h>
#include <omp.h>
#include <iterator>
#include <chrono>
#include <pcl/filters/extract_indices.h>

#include "cloud.h"
#include "display_normals.h"
#include "pre_process.h"
#include "save_vector.h"
#include "save_axis.h"
#include "get_translation.h"
#include "get_rotation.h"
#include "get_LCP.h"
#include "save_normals.h"
#include "save_cluster.h"
#include "pcn2pc.h"
#include "filter_normals.h"
#include "get_axis.h"
#include "pre_transform.h"
#include "icp.h"
#include "size_cluster.h"
#include "filter_walls.h"
#include "density_filter.h"


bool comp_clus(Cluster clus1, Cluster clus2 );


typedef pcl::PointXYZ pcl_point;

int main(int argc, char *argv[])
{

    if(argc<10)
    {
        std::cout<<"normal usage :"<<std::endl;
        std::cout<<"file1  file2 sample_coeff radius_for_normals  angle_for_initial_rotation density_threshold_to_filter_normals density_radius_to_filter_normals number_points_meanshifts bin_width_for_translation uniform_filter_paramter_for_LCP"<<std::endl<<std::endl;
    }
    ///preprocess clouds--------------------------------------------------------------------------------------------------------------
    ///

    std::string output=argv[10];

    auto t_tot1 = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_tgt (new pcl::PointCloud<pcl::Normal>);

    Eigen::Matrix4f transform_init = Eigen::Matrix4f::Identity();
    float theta_init =  atof(argv[5])*M_PI/180;
    std::vector<float> rot_axis={1, 0, 0};

    float sample =atof(argv[3]);
    float normal_radius=atof(argv[4]);
    double reso1;
    double reso2;
    pre_process(argv[1],sample, normal_radius, 0, cloud_src, transform_init, normals_src, &reso1);
    pre_transform(theta_init,rot_axis,&transform_init);
    pre_process(argv[2],sample, normal_radius, 0, cloud_tgt,  transform_init, normals_tgt, &reso2);

//    pcl::io::savePCDFileASCII ("preprocess_src.pcd", *cloud_src);
//    pcl::io::savePCDFileASCII ("preprocess_tgt.pcd", *cloud_tgt);

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt(new pcl::PointCloud<pcl::PointNormal>);

    pcl::concatenateFields (*cloud_src, *normals_src, *pointNormals_src);
    pcl::concatenateFields (*cloud_tgt, *normals_tgt, *pointNormals_tgt);

    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_src, *pointNormals_src, indices);
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_tgt, *pointNormals_tgt, indices);

    std::cout<< "source: points number after preprocessing : "<<pointNormals_src->size()<<std::endl;
    std::cout<< "target: points number after preprocessing : "<<pointNormals_tgt->size()<<std::endl<<std::endl;

    ///filter points to get points with normal laying in clusters
    ///
    //1_create a pointcloud representing normals as points on a sphere

    pcl::PointCloud<pcl_point>::Ptr normals1(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl_point>::Ptr normals2(new pcl::PointCloud<pcl_point>);
    pcn2pc(pointNormals_src, normals1);
    pcn2pc(pointNormals_tgt, normals2);

    //2_ filter normals to enhance clusters   HEAVY STEP

    auto t_filter_1 = std::chrono::high_resolution_clock::now();
//    pcl::io::savePCDFileASCII ("normals1_before.pcd", *normals1);
//    pcl::io::savePCDFileASCII ("normals2_before.pcd", *normals2);

    std::cout<<"cloud source: normal points kept for filtering by density: "<<normals1->points.size()<<std::endl;
    std::cout<<"cloud target: normal points kept for filtering by density : "<<normals2->points.size()<<std::endl<<std::endl;

    float radius =atof(argv[6]);
    int keep =atoi(argv[7]);

//    density_filter(normals1, radius, (int)neigh/2);
    density_filter(normals1, radius, keep);


//    density_filter(normals2, radius, (int)neigh/2);
    density_filter(normals2, radius, keep);

//    pcl::io::savePCDFileASCII ("normals1_after.pcd", *normals1);
//    pcl::io::savePCDFileASCII ("normals2_after.pcd", *normals2);



//to look all points with a normal further from the mode of the cluster

//    pcl::PointCloud<pcl::PointNormal>::Ptr points_weird_normals(new pcl::PointCloud<pcl::PointNormal>);
//    pcl::PointCloud<pcl_point>::Ptr normals2_filtered(new pcl::PointCloud<pcl_point>);
//    pcl::RadiusOutlierRemoval<pcl_point> rorfilter (true);
//    rorfilter.setInputCloud (normals2);
//    rorfilter.setRadiusSearch (0.01);
//    rorfilter.setMinNeighborsInRadius (floor(normals2->points.size()*perc )  );
//    rorfilter.setNegative (true);
//    rorfilter.filter (*normals2_filtered);
//    pcl::IndicesConstPtr rem_ind = rorfilter.getRemovedIndices();
//    pcl::copyPointCloud(*pointNormals_tgt,*rem_ind, *points_weird_normals);

//    pcl::io::savePCDFileASCII ("normals2_after_filtered.pcd", *normals2_filtered);
//    pcl::io::savePCDFileASCII ("weird_normals.pcd", *points_weird_normals);




    auto t_filter_2 = std::chrono::high_resolution_clock::now();
    std::cout<<"total time to filter normals :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_filter_2-t_filter_1).count()<<" milliseconds"<<std::endl<<std::endl;

    ///find clusters with meanshift computation
    auto t_meanshifts1 = std::chrono::high_resolution_clock::now();

    std::cout<<"cloud source: normal points kept for meanshift : "<<normals1->points.size()<<std::endl;
    std::cout<<"cloud target: normal points kept for meanshift : "<<normals2->points.size()<<std::endl<<std::endl;

    std::vector< vector<double> > vec_normals_src(normals1->points.size(), std::vector<double>(3, 0.0));
    std::vector< vector<double> > vec_normals_tgt(normals2->points.size(), std::vector<double>(3, 0.0));

    double kernel_bandwidth = 0.1;
    MeanShift *msp = new MeanShift();
    for (int i=0; i<normals1->points.size(); i++)
    {
        vec_normals_src[i][0]=normals1->points[i].x;
        vec_normals_src[i][1]=normals1->points[i].y;
        vec_normals_src[i][2]=normals1->points[i].z;
    }

    for (int i=0; i<normals2->points.size(); i++)
    {
        vec_normals_tgt[i][0]=normals2->points[i].x;
        vec_normals_tgt[i][1]=normals2->points[i].y;
        vec_normals_tgt[i][2]=normals2->points[i].z;
    }
    vector<Cluster> clusters1 = msp->cluster(vec_normals_src, kernel_bandwidth);
    vector<Cluster> clusters2 = msp->cluster(vec_normals_tgt, kernel_bandwidth);

    std::cout<<"number of clusters found in source : "<<clusters1.size()<<std::endl;
    std::cout<<"number of clusters found in target : "<<clusters2.size()<<std::endl<<std::endl;

    auto t_meanshifts2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration_cast<std::chrono::milliseconds>(t_meanshifts2-t_meanshifts1).count();
    std::cout<<"total time to get clusters with meanshift :" << std::chrono::duration_cast<std::chrono::milliseconds>(t_meanshifts2-t_meanshifts1).count()<<" milliseconds"<<std::endl<<std::endl;

    for (int i=0; i<clusters1.size(); i++)
    {
        float size=size_cluster(clusters1[i]);
        if(size<0.97)
        {
            clusters1[i].original_points.resize(0);
        }
    }

    for (int i=0; i<clusters2.size(); i++)
    {
        float size=size_cluster(clusters2[i]);
        if(size<0.97)
        {
            clusters2[i].original_points.resize(0);
        }
    }

    ///sort clusters and keep only the 6th most important

    sort(clusters1.begin(), clusters1.end(),comp_clus);
    sort(clusters2.begin(), clusters2.end(),comp_clus);

//    int n_clus1=std::min(6,(int)clusters1.size());
//    int n_clus2=std::min(6,(int)clusters2.size());

//    vector<Cluster> clusters1_test;
//    vector<Cluster> clusters2_test;
//    for(int i=0; i<n_clus1; i++)
//    {
//        clusters1_test.push_back(clusters1[i]);
//    }
//    for(int i=0; i<n_clus2; i++)
//    {
//        clusters2_test.push_back(clusters2[i]);
//    }
//    clusters1.clear();
//    clusters1=clusters1_test;
//    clusters2.clear();
//    clusters2=clusters2_test;

    for(int clus = 0; clus < clusters1.size(); clus++)
    {
        float modu=sqrt(clusters1[clus].mode[0]*clusters1[clus].mode[0]+clusters1[clus].mode[1]*clusters1[clus].mode[1]+clusters1[clus].mode[2]*clusters1[clus].mode[2]);
        for (int m=0; m<3; m++)
        {
            clusters1[clus].mode[m]=clusters1[clus].mode[m]/(modu);
        }
    }

    for(int clus = 0; clus < clusters2.size(); clus++)
    {
        float modu=sqrt(clusters2[clus].mode[0]*clusters2[clus].mode[0]+clusters2[clus].mode[1]*clusters2[clus].mode[1]+clusters2[clus].mode[2]*clusters2[clus].mode[2]);
        for (int m=0; m<3; m++)
        {
            clusters2[clus].mode[m]=clusters2[clus].mode[m]/(modu);
        }
    }

    ///save normals

    for(int clus = 0; clus < clusters1.size(); clus++)
    {
        std::stringstream sstm;
        sstm<<"cluster1_mode"<<clus<<".csv";
        std::string cluster_name = sstm.str();
         save_cluster(clusters1[clus].mode,cluster_name);
    }

    for(int clus = 0; clus < clusters2.size(); clus++)
    {
        std::stringstream sstm;
        sstm<<"cluster2_mode"<<clus<<".csv";
        std::string cluster_name = sstm.str();
         save_cluster(clusters2[clus].mode,cluster_name);
    }

    /// keep all normals in a vector

    std::vector<std::vector<double>> vec_normals1(clusters1.size(), std::vector<double>(3));
    std::vector<std::vector<double>> vec_normals2(clusters2.size(), std::vector<double>(3));

    for(int clus = 0; clus < clusters1.size(); clus++)
    {
        vec_normals1[clus]=clusters1[clus].mode;
    }

    for(int clus = 0; clus < clusters2.size(); clus++)
    {
        vec_normals2[clus]=clusters2[clus].mode;
    }

    /// filter pointclouds to keep only planar sections (which are used in translation process)

    filter_walls(pointNormals_src, vec_normals1);
    filter_walls(pointNormals_tgt, vec_normals2);

    ///--------------------------------Loop to test all pairs combination between source and target----------------------------------------------------------------------


    //build pairs with non parallel normals

    std::vector< pair<int,int> > pairs1;
    for (int q=0; q<clusters1.size()-1; q++)
    {
        for (int p=q+1; p<clusters1.size(); p++)
        {
          if( abs(clusters1[p].mode[0]*clusters1[q].mode[0] + clusters1[p].mode[1]*clusters1[q].mode[1] + clusters1[p].mode[2]*clusters1[q].mode[2])<0.9)
            pairs1.push_back(make_pair(q,p));
        }
    }

    std::vector< pair<int,int> > pairs2;
    for (int q=0; q<clusters2.size(); q++)
    {
        for (int p=0; p<clusters2.size(); p++)
        {
          if( abs(clusters2[p].mode[0]*clusters2[q].mode[0] + clusters2[p].mode[1]*clusters2[q].mode[1] + clusters2[p].mode[2]*clusters2[q].mode[2])<0.9 && p!=q)
            pairs2.push_back(make_pair(q,p));
        }
    }

    std::cout<<"number of pairs of cloud 1 : "<<pairs1.size()<<std::endl;
    std::cout<<"number of pairs of cloud 2 : "<<pairs2.size()<<std::endl<<std::endl;
    std::cout<<"number of trials : "<<pairs1.size()*pairs2.size()<<std::endl<<std::endl;

    float bin_width=atof(argv[8]);
    float lim = 0.99;
    std::vector<int> LCP_vec(pairs1.size()*clusters2.size()*clusters2.size());
    std::vector<Eigen::Matrix4f> total_transform_vec(pairs1.size()*clusters2.size()*clusters2.size());
    Eigen::Matrix4f good_transform = Eigen::Matrix4f::Identity();

    //sample clouds to speed up transform and LCP calculation

    float LCP_samp = atof(argv[9]);
    pcl::UniformSampling<pcl_point> uniform_sampling;
    uniform_sampling.setRadiusSearch (LCP_samp);
    uniform_sampling.setInputCloud (cloud_src);
    uniform_sampling.filter (*cloud_src);
    uniform_sampling.setInputCloud (cloud_tgt);
    uniform_sampling.filter (*cloud_tgt);

    pcl::KdTreeFLANN<pcl_point>::Ptr tree_tgt (new pcl::KdTreeFLANN<pcl_point>);
    tree_tgt->setInputCloud(cloud_tgt);

    std::cout<<"points number for LCP calculation :"<< cloud_src->points.size()<<"  "<<cloud_tgt->points.size()<<std::endl<<std::endl;

    auto t_loop1= std::chrono::high_resolution_clock::now();

    #pragma omp parallel for schedule(dynamic) firstprivate(pairs1, bin_width, lim, clusters1, clusters2, pointNormals_src, pointNormals_tgt, cloud_src, tree_tgt ) shared( LCP_vec, total_transform_vec )


    for (int w=0; w<pairs1.size(); w++)
    {
        int q=pairs1[w].first;
        int p=pairs1[w].second;

        vector<vector<double>> walls1(2);

        walls1[0]=clusters1[q].mode;
        walls1[1]=clusters1[p].mode;

        float dot=walls1[0][0]*walls1[1][0]+walls1[0][1]*walls1[1][1]+walls1[0][2]*walls1[1][2]; // allows to check if the angle between pairs is the same
        float al1=acos(dot);

        for (int m=0; m<pairs2.size(); m++)
        {
            int r=pairs2[m].first;
            int s=pairs2[m].second;
            vector<vector<double>> walls2(2);
            walls2[0]=clusters2[r].mode;
            walls2[1]=clusters2[s].mode;
            dot=walls2[0][0]*walls2[1][0]+walls2[0][1]*walls2[1][1]+walls2[0][2]*walls2[1][2];
            float al2=acos(dot);


            /// get main walls and their normal to perform translation search. these normals represent translation axis


                //choisir le nuage qui a le moins de clusters comme target
                std::vector<std::vector<float>> axis(3, std::vector<float>(3, 0.0));

                for(int i=0; i<3; i++)
                {
                    axis[0][i]=walls2[0][i];
                    axis[1][i]=walls2[1][i];
                }

                bool error;
                get_axis(clusters2, axis, &error);

                if(error)
                {
                    std::cout<<"error : not enough normals to get clusters, please increase number of normals"<<std::endl<<std::endl;
                    break;
                }


            int LCP=0;

            if (abs(al1-al2)<0.05)  //checking if angles are the same
            {
                pcl::PointCloud<pcl::PointNormal> source;
                pcl::PointCloud<pcl::PointNormal> target;

                pcl::copyPointCloud(*pointNormals_src,source);
                pcl::copyPointCloud(*pointNormals_tgt,target);
                Eigen::Matrix4f rotation_transform = Eigen::Matrix4f::Identity();
                get_rotation(walls1, walls2, &rotation_transform);
                pcl::transformPointCloudWithNormals (source, source, rotation_transform);           

                ///get translation with histograms correlation

                Eigen::Matrix4f translation_transform = Eigen::Matrix4f::Zero();
                pcl::PointCloud<pcl::PointNormal>::Ptr source_ptr(new pcl::PointCloud<pcl::PointNormal>);
                pcl::PointCloud<pcl::PointNormal>::Ptr target_ptr(new pcl::PointCloud<pcl::PointNormal>);
                *source_ptr=source;
                *target_ptr=target;
                get_translation(source_ptr, target_ptr, lim, axis, bin_width, &translation_transform) ;

                Eigen::Matrix4f total_transform = Eigen::Matrix4f::Zero();
                total_transform=rotation_transform+translation_transform;

                ///compute LCP for this transformation
                get_LCP(cloud_src, tree_tgt, LCP_samp, &total_transform, &LCP);
                int lim_for=clusters2.size();
                LCP_vec[w*lim_for*lim_for+r*lim_for+s]=LCP;
                total_transform_vec[w*lim_for*lim_for+r*lim_for+s]=total_transform;
            }

          }
    }

    auto t_loop2 = std::chrono::high_resolution_clock::now();
    std::cout<<"time for the loop :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_loop2-t_loop1).count()<<" milliseconds"<<std::endl<<std::endl;

    ///get best transform using LCP value

    int index=0;

    int temp=0;
    for (int i=0; i<LCP_vec.size(); i++)
    {
        if(temp<LCP_vec[i])
        {
            temp=LCP_vec[i];
            index=i;
        }
    }

    good_transform=total_transform_vec[index];
    std::cout<<"best transformation :"<<std::endl<<good_transform<<std::endl<<std::endl;

    auto t_meth = std::chrono::high_resolution_clock::now();
    std::cout<<"time to get transform with our method :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_meth-t_tot1).count()<<" milliseconds"<<std::endl<<std::endl;

    ///do one icp step-----------------------------------------------------------------------------------------------------------------------------------------------------

//    pcl::UniformSampling<pcl::PointNormal> uniform_sampling_n;
//    uniform_sampling_n.setRadiusSearch (sample*2);
//    uniform_sampling_n.setInputCloud (pointNormals_src);
//    uniform_sampling_n.filter (*pointNormals_src);
//    uniform_sampling_n.setInputCloud (pointNormals_tgt);
//    uniform_sampling_n.filter (*pointNormals_tgt);

//    std::cout<<"points number for icp: "<<pointNormals_src->size()<<std::endl;
//    std::cout<<"points number for icp: "<<pointNormals_tgt->size()<<std::endl<<std::endl;

//    pcl::transformPointCloudWithNormals (*pointNormals_src, *pointNormals_src, good_transform);
//    Eigen::Matrix4f icp_transform;
//    Eigen::Matrix4f final_transform;
//    icp(pointNormals_src, pointNormals_tgt, 0.05, 0.4, 50, &icp_transform);
//    final_transform=icp_transform*good_transform;

//    auto t_tot2 = std::chrono::high_resolution_clock::now();
//    std::cout<<"total time to get transform with icp refinement :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_tot2-t_tot1).count()<<" milliseconds"<<std::endl<<std::endl;

    ///save best transformation in build/results/transformations and the time

    std::stringstream sstm;
    sstm.str("");
    sstm<<"mkdir /home/julia/Desktop/my_programs/hist_registration/build-hist_registration/results/transformations/"<<output;
    std::string command = sstm.str();
    const char* c=command.c_str();
    system(c);

        std::string file_name;
        std::string file_name1;
        std::string file_name2;
        file_name=argv[1];
        size_t lastindex_point = file_name.find_last_of(".");
        size_t lastindex_slash = file_name.find_last_of("/");
        if (lastindex_slash==std::string::npos)
        {
           lastindex_slash = 0;
        }

        file_name1 = file_name.substr(lastindex_slash+1, lastindex_point-(lastindex_slash+1));
        file_name=argv[2];
        lastindex_point = file_name.find_last_of(".");
        lastindex_slash = file_name.find_last_of("/");
        if (lastindex_slash==std::string::npos)
        {
           lastindex_slash = 0;
        }
        file_name2 = file_name.substr(lastindex_slash+1, lastindex_point-(lastindex_slash+1));

        sstm.str("");
        sstm<<"/home/julia/Desktop/my_programs/hist_registration/build-hist_registration/results/transformations/"<<output<<"/"<<file_name1<<"_"<<file_name2<<".txt";
        std::string file_name_tot = sstm.str();
        ofstream file (file_name_tot);
        file<<good_transform;
//        file<<"\n";
//        file<<final_transform;
        file<<"\n";
        file<<std::chrono::duration_cast<std::chrono::milliseconds>(t_meth-t_tot1).count();
        file<<"\n";
        file<<std::chrono::duration_cast<std::chrono::milliseconds>(t_meanshifts2-t_meanshifts1).count();
        file<<"\n";
        file<<reso1<<" "<<reso2;
        file.close();

//        std::cout<<"result transformation :"<<std::endl<<final_transform<<std::endl<<std::endl;

        file.open("/home/julia/Desktop/my_programs/hist_registration/build-hist_registration/results/Times", std::ofstream::out | std::ofstream::app);
//        file<<"\n"<<"\n"<<"\n"
//            <<"file1: "<<argv[1]<<" and "
//            <<"file2: "<<argv[2]<<std::endl
//            <<"sample_coeff: "<<argv[3]<<std::endl
//            <<"angle_for_initial_rotation: "<<argv[5]<<std::endl
//            <<"normal_filter_parameter: "<<argv[6]<<std::endl
//            <<"bin_width_for_translation: "<<argv[7]<<std::endl
//            <<"TIME: "<<std::chrono::duration_cast<std::chrono::milliseconds>(t_tot2-t_tot1).count();
        file.close();


}

bool comp_clus(Cluster clus1, Cluster clus2 )
{
    if ( clus1.original_points.size()>clus2.original_points.size() )
    {
        return true;
    }
    return false;
}

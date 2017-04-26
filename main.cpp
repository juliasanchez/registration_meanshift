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




bool comp_clus(Cluster clus1, Cluster clus2 );


typedef pcl::PointXYZ pcl_point;

int main(int argc, char *argv[])
{
    if(argc<11)
    {
        std::cout<<"normal usage :"<<std::endl;
        std::cout<<"file1  file2 sample_coeff  angle_for_initial_rotation density_threshold_to_filter_normals  radius_to_find_highest_density_point bin_width_for_translation filter_parameter_to_keep_walls[0,1] normals_to_keep_number RANSAC_thresh_for_icp max_dist_for_icp it_for_icp"<<std::endl<<std::endl;
    }
    ///preprocess clouds--------------------------------------------------------------------------------------------------------------

    auto t_tot1 = std::chrono::high_resolution_clock::now();

    float sample =atof(argv[3]);
    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_tgt (new pcl::PointCloud<pcl::Normal>);

    Eigen::Matrix4f transform_init = Eigen::Matrix4f::Identity();
    float theta_init =  atof(argv[4])*M_PI/180;
    std::vector<float> rot_axis={1, 0, 0};

    pre_process(argv[1],sample, 0, cloud_src, transform_init, normals_src);
    pre_transform(theta_init,rot_axis,&transform_init);
    pre_process(argv[2],sample, 0, cloud_tgt,  transform_init, normals_tgt);

    pcl::io::savePCDFileASCII ("preprocess_src.pcd", *cloud_src);
    pcl::io::savePCDFileASCII ("preprocess_tgt.pcd", *cloud_tgt);

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
    float radius =0.02;
    float perc = atof(argv[5]);
    pcl::io::savePCDFileASCII ("normals1_before.pcd", *normals1);
    pcl::io::savePCDFileASCII ("normals2_before.pcd", *normals2);

    filter_normals(normals1, radius, perc);
    filter_normals(normals2, radius, perc);



//    pcl::RadiusOutlierRemoval<pcl_point> rorfilter(true);
//    rorfilter.setInputCloud (normals2);
//    rorfilter.setRadiusSearch (0.02);
//    rorfilter.setMinNeighborsInRadius (floor(normals2->points.size()*perc )  );
//    rorfilter.filter (*normals2);
//    pcl::IndicesConstPtr removed_ind;
//    removed_ind = rorfilter.getRemovedIndices ();



//    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt_filtered(new pcl::PointCloud<pcl::PointNormal>);
//    pcl::ExtractIndices<pcl::PointNormal> eifilter; // Initializing with true will allow us to extract the removed indices
//    eifilter.setNegative (true);
//    eifilter.setInputCloud (pointNormals_tgt);
//    eifilter.setIndices (removed_ind);
//    eifilter.filter (*pointNormals_tgt_filtered);
//    pcl::io::savePCDFileASCII ("points_in_clusters.pcd", *pointNormals_tgt_filtered);





    pcl::io::savePCDFileASCII ("normals1_after.pcd", *normals1);
    pcl::io::savePCDFileASCII ("normals2_after.pcd", *normals2);

    auto t_filter_2 = std::chrono::high_resolution_clock::now();
    std::cout<<"total time to filter normals :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_filter_2-t_filter_1).count()<<" milliseconds"<<std::endl<<std::endl;


    ///find clusters with meanshift computation
    auto t_meanshifts1 = std::chrono::high_resolution_clock::now();

    int N1_normals=std::min((int)(normals1->points.size()),atoi(argv[8]));
    int N2_normals=std::min((int)(normals2->points.size()),atoi(argv[8]));

    int samp1=(int)(normals1->points.size()/N1_normals);
    int samp2=(int)(normals2->points.size()/N2_normals);
    std::cout<<"cloud source: points number : "<<normals1->points.size()<<std::endl;
    std::cout<<"cloud target: points number : "<<normals2->points.size()<<std::endl<<std::endl;
    std::cout<<"cloud source: points kept : "<<normals1->points.size()/samp1<<std::endl;
    std::cout<<"cloud target: points kept : "<<normals2->points.size()/samp2<<std::endl<<std::endl;
    double kernel_bandwidth = 0.1;

    MeanShift *msp = new MeanShift();
    std::vector< vector<double> > vec_normals_src((int)(normals1->points.size()/samp1)+1, std::vector<double>(3, 0.0));
    std::vector< vector<double> > vec_normals_tgt((int)(normals2->points.size()/samp2)+1, std::vector<double>(3, 0.0));
    int n=0;
    for (int i=0; i<normals1->points.size(); i=i+samp1)
    {
        vec_normals_src[n][0]=normals1->points[i].x;
        vec_normals_src[n][1]=normals1->points[i].y;
        vec_normals_src[n][2]=normals1->points[i].z;
        n++;
    }
    n=0;
    for (int i=0; i<normals2->points.size(); i=i+samp2)
    {
        vec_normals_tgt[n][0]=normals2->points[i].x;
        vec_normals_tgt[n][1]=normals2->points[i].y;
        vec_normals_tgt[n][2]=normals2->points[i].z;
        n++;
    }
    vector<Cluster> clusters1 = msp->cluster(vec_normals_src, kernel_bandwidth);
    vector<Cluster> clusters2 = msp->cluster(vec_normals_tgt, kernel_bandwidth);

    std::cout<<"number of clusters found in source : "<<clusters1.size()<<std::endl;
    std::cout<<"number of clusters found in target : "<<clusters2.size()<<std::endl<<std::endl;

    auto t_meanshifts2 = std::chrono::high_resolution_clock::now();
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

    int n_clus1=std::min(6,(int)clusters1.size());
    int n_clus2=std::min(6,(int)clusters2.size());

    vector<Cluster> clusters1_test;
    vector<Cluster> clusters2_test;
    for(int i=0; i<n_clus1; i++)
    {
        if(clusters1[i].original_points.size()!=0)
        clusters1_test.push_back(clusters1[i]);
    }
    for(int i=0; i<n_clus2; i++)
    {
        if(clusters2[i].original_points.size()!=0)
        clusters2_test.push_back(clusters2[i]);
    }
    clusters1.clear();
    clusters1=clusters1_test;
    clusters2.clear();
    clusters2=clusters2_test;

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

  /// get main walls and their normal to perform translation search. these normals become axis

      //choisir le nuage qui a le moins de clusters comme target
      std::vector<std::vector<float>> axis(3, std::vector<float>(3, 0.0));
      std::vector<Cluster> clusters;

      bool modif_axis;
      if(clusters1.size()>=clusters2.size())
      {
          clusters=clusters2;
          modif_axis=false;
      }
      else
        {
          clusters=clusters1;
          modif_axis=true;
        }

      bool error;
      get_axis(clusters, axis, &error);

      save_axis(axis[0], "axis_x.csv");
      save_axis(axis[1], "axis_y.csv");
      save_axis(axis[2], "axis_z.csv");

      if(error)
      {
          std::cout<<"error : not enough normals to get clusters, please increase number of normals"<<std::endl<<std::endl;
          return 0;
      }

    ///--------------------------------Loop to test all pairs combination between source and target----------------------------------------------------------------------

    std::vector< pair<int,int> > pairs1;
    for (int q=0; q<clusters1.size()-1; q++)
    {
        for (int p=q+1; p<clusters1.size(); p++)
        {
          if( abs(clusters1[p].mode[0]*clusters1[q].mode[0] + clusters1[p].mode[1]*clusters1[q].mode[1] + clusters1[p].mode[2]*clusters1[q].mode[2])<0.7)
            pairs1.push_back(make_pair(q,p));
        }
    }

    std::vector< pair<int,int> > pairs2;
    for (int q=0; q<clusters2.size(); q++)
    {
        for (int p=0; p<clusters2.size(); p++)
        {
          if( abs(clusters2[p].mode[0]*clusters2[q].mode[0] + clusters2[p].mode[1]*clusters2[q].mode[1] + clusters2[p].mode[2]*clusters2[q].mode[2])<0.7 && p!=q)
            pairs2.push_back(make_pair(q,p));
        }
    }

    std::cout<<"number of pairs of cloud 1 : "<<pairs1.size()<<std::endl;
    std::cout<<"number of pairs of cloud 2 : "<<pairs2.size()<<std::endl<<std::endl;
    std::cout<<"number of trials : "<<pairs1.size()*pairs2.size()<<std::endl<<std::endl;

    float bin_width=atof(argv[6]);
    float lim = atof(argv[7]);
    std::vector<int> LCP_vec(pairs1.size()*clusters2.size()*clusters2.size());
    std::vector<Eigen::Matrix4f> total_transform_vec(pairs1.size()*clusters2.size()*clusters2.size());
    Eigen::Matrix4f good_transform = Eigen::Matrix4f::Identity();

    pcl::UniformSampling<pcl_point> uniform_sampling;
    uniform_sampling.setRadiusSearch (sample*3);
    uniform_sampling.setInputCloud (cloud_src);
    uniform_sampling.filter (*cloud_src);
    uniform_sampling.setInputCloud (cloud_tgt);
    uniform_sampling.filter (*cloud_tgt);

    #pragma omp parallel for schedule(dynamic) firstprivate(pairs1, bin_width, lim, clusters1, clusters2, pointNormals_src, pointNormals_tgt, axis, cloud_src, cloud_tgt ) shared( LCP_vec, total_transform_vec )

    for (int w=0; w<pairs1.size(); w++)
    {
        int q=pairs1[w].first;
        int p=pairs1[w].second;

        vector<vector<double>> walls1(2);

        walls1[0]=clusters1[q].mode;
        walls1[1]=clusters1[p].mode;

        float dot=walls1[0][0]*walls1[1][0]+walls1[0][1]*walls1[1][1]+walls1[0][2]*walls1[1][2];
        float al1=acos(dot);

        ///initialize cloud_src cloud_tgt and phi--------------------------------------------------------------------------------------------------------------------------------------
        for (int m=0; m<pairs2.size(); m++)
        {
            int r=pairs2[m].first;
            int s=pairs2[m].second;
            int LCP1=0;
            int LCP2=0;
            int LCP3=0;
            vector<vector<double>> walls2(2);
            walls2[0]=clusters2[r].mode;
            walls2[1]=clusters2[s].mode;

            dot=walls2[0][0]*walls2[1][0]+walls2[0][1]*walls2[1][1]+walls2[0][2]*walls2[1][2];
            float al2=acos(dot);
            if (abs(al1-al2)<0.2)
            {

                pcl::PointCloud<pcl::PointNormal> source;
                pcl::PointCloud<pcl::PointNormal> target;

                pcl::copyPointCloud(*pointNormals_src,source);
                pcl::copyPointCloud(*pointNormals_tgt,target);

                ///rotate cloud in the accurate way with rotation information--------------------------------------------------------------------------------------------------------------------------------------
                Eigen::Matrix4f rotation_transform = Eigen::Matrix4f::Identity();
                get_rotation(walls1, walls2, &rotation_transform);

                pcl::transformPointCloudWithNormals (source, source, rotation_transform);
                std::vector<std::vector<float>> temp(3, std::vector<float>(3, 0.0));
                temp=axis;
                ///TURN AXIS IF THEY COME FROM SOURCE
                if(modif_axis)
                {
                    for (int h=0; h<3; h++)
                    {
                        for (int g=0; g<3; g++)
                        {
                            temp[h][g]=axis[h][0]*rotation_transform(g,0)+axis[h][1]*rotation_transform(g,1)+axis[h][2]*rotation_transform(g,2);
                        }
                    }
                }

                ///get translation with histograms correlation

                Eigen::Matrix4f translation_transform = Eigen::Matrix4f::Zero();
                pcl::PointCloud<pcl::PointNormal>::Ptr source_ptr(new pcl::PointCloud<pcl::PointNormal>);
                pcl::PointCloud<pcl::PointNormal>::Ptr target_ptr(new pcl::PointCloud<pcl::PointNormal>);
                *source_ptr=source;
                *target_ptr=target;

                get_translation(source_ptr, target_ptr, lim, temp, bin_width, &translation_transform) ;

                Eigen::Matrix4f total_transform = Eigen::Matrix4f::Zero();
                total_transform=rotation_transform+translation_transform;

                ///gcompute LCP for this transformation
                get_LCP(*cloud_src, *cloud_tgt, 0.05, &total_transform, &LCP1);
                get_LCP(*cloud_src, *cloud_tgt, 0.08, &total_transform, &LCP2);
                get_LCP(*cloud_src, *cloud_tgt, 0.02, &total_transform, &LCP3);
                int lim_for=clusters2.size();
                LCP_vec[w*lim_for*lim_for+r*lim_for+s]=LCP1+LCP2+LCP3;
                total_transform_vec[w*lim_for*lim_for+r*lim_for+s]=total_transform;
            }

          }
    }


    ///get best transform using LCP value

    int index=0;
    int temp=0;
    for (int i=0; i<LCP_vec.size(); i++)
    {
        if(LCP_vec[i]>temp)
        {
            temp=LCP_vec[i];
            index=i;
        }
    }
    good_transform=total_transform_vec[index];
    std::cout<<"best transformation :"<<std::endl<<good_transform<<std::endl<<std::endl;

    auto t_meth = std::chrono::high_resolution_clock::now();
    std::cout<<"time to get transform with our method :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_meth-t_tot1).count()<<" milliseconds"<<std::endl<<std::endl;

    ///do one icp step

    pcl::UniformSampling<pcl::PointNormal> uniform_sampling_n;
    uniform_sampling_n.setRadiusSearch (sample*2);
    uniform_sampling_n.setInputCloud (pointNormals_src);
    uniform_sampling_n.filter (*pointNormals_src);
    uniform_sampling_n.setInputCloud (pointNormals_tgt);
    uniform_sampling_n.filter (*pointNormals_tgt);

    std::cout<<"points number for icp: "<<pointNormals_src->size()<<std::endl;
    std::cout<<"points number for icp: "<<pointNormals_tgt->size()<<std::endl<<std::endl;

    pcl::transformPointCloudWithNormals (*pointNormals_src, *pointNormals_src, good_transform);
    Eigen::Matrix4f icp_transform;
    Eigen::Matrix4f final_transform;
    icp(pointNormals_src, pointNormals_tgt, atof(argv[9]), atof(argv[10]), atoi(argv[11]), &icp_transform);
    final_transform=icp_transform*good_transform;

    auto t_tot2 = std::chrono::high_resolution_clock::now();
    std::cout<<"total time to get transform with icp refinement :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_tot2-t_tot1).count()<<" milliseconds"<<std::endl<<std::endl;

    ///save best transformation in build/results/transformations and the time
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
        std::stringstream sstm;
        sstm.str("");
        sstm<<"/home/julia/Desktop/my_programs/hist_registration/build-hist_registration/results/transformations/"<<file_name1<<"_"<<file_name2<<".txt";
        std::string file_name_tot = sstm.str();
        ofstream file (file_name_tot);
        file<<good_transform;
        file<<"\n";
        file<<final_transform;
        file<<"\n";
        file<<std::chrono::duration_cast<std::chrono::milliseconds>(t_tot2-t_tot1).count();
        file.close();

        std::cout<<"result transformation :"<<std::endl<<final_transform<<std::endl<<std::endl;

        file.open("/home/julia/Desktop/my_programs/hist_registration/build-hist_registration/results/Times", std::ofstream::out | std::ofstream::app);
        file<<"\n"<<"\n"<<"\n"
            <<"file1: "<<argv[1]<<" and "
            <<"file2: "<<argv[2]<<std::endl
            <<"sample_coeff: "<<argv[3]<<std::endl
            <<"angle_for_initial_rotation: "<<argv[4]<<std::endl
            <<"normal_filter_parameter: "<<argv[5]<<std::endl
            <<"bin_width_for_translation: "<<argv[6]<<std::endl
            <<"filter_parameter_to_keep_walls: "<<argv[7]<<std::endl
            <<"sample_coeff_for_meanshift: "<<argv[8]<<std::endl
            <<"TIME: "<<std::chrono::duration_cast<std::chrono::milliseconds>(t_tot2-t_tot1).count();
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

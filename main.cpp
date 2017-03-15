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

#include "cloud.h"
#include "display_normals.h"
#include "pre_process.h"
#include "save_vector.h"
#include "save_axis.h"
#include "get_translation.h"
#include "get_LCP.h"
#include "extract_spots.h"
#include "MeanShift.h"
#include "save_normals.h"
#include "save_cluster.h"
#include "fact.h"


typedef pcl::PointXYZ pcl_point;

int main(int argc, char *argv[])
{
    ///preprocess clouds--------------------------------------------------------------------------------------------------------------

    clock_t t_tot=clock();
    float f=CLOCKS_PER_SEC;

    float normal_radius =atof(argv[4]);
    float sample =atof(argv[3]);
    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt(new pcl::PointCloud<pcl_point>);

    pcl::PointCloud<pcl::Normal>::Ptr normals_tgt (new pcl::PointCloud<pcl::Normal>);

    Eigen::Matrix4f transform_init = Eigen::Matrix4f::Identity();

    pre_process(argv[1],sample,normal_radius, 0, 10, cloud_src, transform_init, normals_src);

//    std::vector<int> indices;
//    pcl::removeNaNNormalsFromPointCloud(*normals_src, *normals_src, indices);
//    pcl::io::savePCDFileASCII ("normals.pcd", *normals_src);

    Eigen::Matrix4f transform_phi_init = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_theta_init = Eigen::Matrix4f::Identity();

    float theta_init =  atof(argv[8])*M_PI/180;
    float phi_init=  atof(argv[9])*M_PI/180;
    std::vector<float> rot_ax={cloud_src->points[0].y, -cloud_src->points[0].x,0};
    transform_theta_init  (0,0) = rot_ax[0]*rot_ax[0]*(1-cos (theta_init))+cos (theta_init);
    transform_theta_init  (0,1) = rot_ax[0]*rot_ax[1]*(1-cos (theta_init))-rot_ax[2]*sin (theta_init);
    transform_theta_init  (0,2) = rot_ax[0]*rot_ax[2]*(1-cos (theta_init))+rot_ax[1]*sin (theta_init);
    transform_theta_init  (1,0) = rot_ax[1]*rot_ax[0]*(1-cos (theta_init))+rot_ax[2]*sin (theta_init);
    transform_theta_init  (1,1) = rot_ax[1]*rot_ax[1]*(1-cos (theta_init))+cos (theta_init);
    transform_theta_init  (1,2) = rot_ax[1]*rot_ax[2]*(1-cos (theta_init))-rot_ax[0]*sin (theta_init);
    transform_theta_init  (2,0) = rot_ax[2]*rot_ax[0]*(1-cos (theta_init))-rot_ax[1]*sin (theta_init);
    transform_theta_init  (2,1) = rot_ax[2]*rot_ax[1]*(1-cos (theta_init))+rot_ax[0]*sin (theta_init);
    transform_theta_init  (2,2) = rot_ax[2]*rot_ax[2]*(1-cos (theta_init))+cos (theta_init);

    transform_phi_init  (0,0) = cos (phi_init);
    transform_phi_init  (0,1) = -sin(phi_init);
    transform_phi_init  (1,0) = sin(phi_init);
    transform_phi_init  (1,1) = cos(phi_init);


    transform_init=transform_phi_init*transform_theta_init;

    // Print the transformation
    std::cout << "initial transform : "<<std::endl<<transform_init << std::endl;

    pre_process(argv[2],sample,normal_radius, 0, 10, cloud_tgt,  transform_init, normals_tgt);


    pcl::io::savePCDFileASCII ("preprocess_src.pcd", *cloud_src);
    pcl::io::savePCDFileASCII ("preprocess_tgt.pcd", *cloud_tgt);

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal> pointNormals_src_init;
    pcl::PointCloud<pcl::PointNormal> pointNormals_tgt_init;
    pcl::concatenateFields (*cloud_src, *normals_src, *pointNormals_src);
    pcl::concatenateFields (*cloud_tgt, *normals_tgt, *pointNormals_tgt);

    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_src, *pointNormals_src, indices);
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_tgt, *pointNormals_tgt, indices);

    pcl::io::savePCDFileASCII ("src_with_normals.pcd", *pointNormals_src);
    pcl::io::savePCDFileASCII ("tgt_with_normals.pcd", *pointNormals_tgt);

    pointNormals_src_init=*pointNormals_src;
    pointNormals_tgt_init=*pointNormals_tgt;


//    std::vector<std::vector<float>> hist1_angles((int)(N/2), std::vector<float>(N, 0.0));
//    get_angles_hist(pointNormals_src,hist1_angles);
//    float thresh=atof(argv[10]);

//    std::vector<std::vector<float> > spots1;
//    extract_spots(hist1_angles, spots1, thresh);

//    std::vector<std::vector<float>> hist2_angles((int)(N/2), std::vector<float>(N, 0.0));
//    get_angles_hist(pointNormals_tgt,hist2_angles);
//    std::vector<std::vector<float> > spots2;
//    extract_spots(hist2_angles, spots2, thresh);




//    int samp=20;

//    MeanShift *msp = new MeanShift();
//    double kernel_bandwidth = 0.2;
//    std::vector< vector<double> > vec_normals_src((int)(pointNormals_src->points.size()/samp)+1, std::vector<double>(3, 0.0));
//    std::vector< vector<double> > vec_normals_tgt((int)(pointNormals_tgt->points.size()/samp)+1, std::vector<double>(3, 0.0));
//    int n=0;
//    for (int i=0; i<pointNormals_src->points.size(); i=i+samp)
//    {
//        vec_normals_src[n][0]=pointNormals_src->points[i].normal_x;
//        vec_normals_src[n][1]=pointNormals_src->points[i].normal_y;
//        vec_normals_src[n][2]=pointNormals_src->points[i].normal_z;
//        n++;
//    }
//    n=0;
//    for (int i=0; i<pointNormals_tgt->points.size(); i=i+samp)
//    {
//        vec_normals_tgt[n][0]=pointNormals_tgt->points[i].normal_x;
//        vec_normals_tgt[n][1]=pointNormals_tgt->points[i].normal_y;
//        vec_normals_tgt[n][2]=pointNormals_tgt->points[i].normal_z;
//        n++;
//    }
//    vector<Cluster> clusters1 = msp->cluster(vec_normals_src, kernel_bandwidth);
//    vector<Cluster> clusters2 = msp->cluster(vec_normals_tgt, kernel_bandwidth);

//    for(int cluster = 0; cluster < clusters1.size(); cluster++)
//    {
//        std::stringstream sstm;
//        sstm<<"cluster1_mode"<<cluster<<".csv";
//        std::string cluster_name = sstm.str();
//         save_cluster(clusters1[cluster].mode,cluster_name);
//    }

//    for(int cluster = 0; cluster < clusters2.size(); cluster++)
//    {
//        std::stringstream sstm;
//        sstm<<"cluster2_mode"<<cluster<<".csv";
//        std::string cluster_name = sstm.str();
//         save_cluster(clusters2[cluster].mode,cluster_name);
//    }

//    ///JUST TO TEST PROGRAM WITHOUT COMPUTING MEANSHIFTS EACH TIME-------------------------------------

    std::vector<Cluster> clusters1(6);
    clusters1[0].mode= {0.483905, 0.869965, -0.000435804};
    clusters1[1].mode= {-0.0102198, -0.0199526, 0.987996};
    clusters1[2].mode= {0.867216, -0.489957, -0.00631862};
    clusters1[3].mode= {-0.47721, -0.866893, 0.00207398};
    clusters1[4].mode= {-0.00156818, 0.00238466, -0.996913};
    clusters1[5].mode= {-0.869894, 0.475461, -0.00944517};
      clusters1[0].original_points.resize(162);
      clusters1[1].original_points.resize(321);
      clusters1[2].original_points.resize(291);
      clusters1[3].original_points.resize(309);
      clusters1[4].original_points.resize(225);
      clusters1[5].original_points.resize(178);

    std::vector<Cluster> clusters2(6);
    clusters2[0].mode= {0.917746, 0.370634, -0.013321};
    clusters2[1].mode= {-0.366446, 0.922714, -0.0148863};
    clusters2[2].mode= {8.38448e-5, 0.00235991, 0.994073};
    clusters2[3].mode= {-0.00664235, 0.00236323, -0.996601};
    clusters2[4].mode= {0.633157, -0.751046, -0.00880965};
    clusters2[5].mode= {-0.0117161, -0.97784, -0.00395968};
      clusters2[0].original_points.resize(210);
      clusters2[1].original_points.resize(384);
      clusters2[2].original_points.resize(576);
      clusters2[3].original_points.resize(441);
      clusters2[4].original_points.resize(67);
      clusters2[5].original_points.resize(89);

      int n_combi1 = fact( clusters1.size() ) / ( fact(2)*fact(clusters1.size()-2) );
      int n_combi2 = fact( clusters2.size() ) / fact(clusters2.size()-2);

  /// get main walls and their normal to perform translation search

      //il faudrait prendre seulement le nuage qui a le moins de cluster
      int temp1=0;
      int temp2=0;

      std::vector<double> norm1(3);
      std::vector<double> norm2(3);
      std::vector<double> norm3(3);

      for (int i=0; i<clusters2.size(); i++)
      {
      double dot=norm1[0]*clusters2[i].mode[0]+norm1[1]*clusters2[i].mode[1]+norm1[2]*clusters2[i].mode[2];
      if(clusters2[i].original_points.size()>temp1 && abs(clusters2[i].mode[2])<0.7) //hypothesis 1 : roof with theta<45°
      {
        if(abs(dot)<0.97)
        {
            temp2=temp1;
            norm2=norm1;
        }
        temp1=clusters2[i].original_points.size();
        norm1=clusters2[i].mode;

      }
      else if(clusters2[i].original_points.size()>temp2 && abs(dot)<0.97 && abs(clusters2[i].mode[2])<0.7 )
      {
        temp2=clusters2[i].original_points.size();
        norm2=clusters2[i].mode;
      }
      }

      norm3[0]=norm1[1]*norm2[2]-norm1[2]*norm2[1];
      norm3[1]=norm1[2]*norm2[0]-norm1[0]*norm2[2];
      norm3[2]=norm1[0]*norm2[1]-norm1[1]*norm2[0]; //hypothesis 2 : walls perpendicular to roof and ground

      std::vector<std::vector<float>> axis(3, std::vector<float>(3, 0.0));
      for (int i=0; i<3; i++)
      {
          axis[0][i]=norm1[i];
          axis[1][i]=norm2[i];
          axis[2][i]=norm3[i];
      }
      save_axis(axis[0], "axis_x.csv");
      save_axis(axis[1], "axis_y.csv");
      save_axis(axis[2], "axis_z.csv");

    ///-------------------------------------------------------------------------------------------------------
    //num_threads(16)

    std::vector< pair<int,int> > pairs;
    for (int q=0; q<clusters1.size()-1; q++)
    {
        for (int p=q+1; p<clusters1.size(); p++)
        {
          pairs.push_back(make_pair(q,p));
        }
    }

    int N_hist_axis=atoi(argv[6]);
    float lim = atof(argv[7]);
    std::vector<int> LCP_vec(pairs.size()*n_combi2);
    std::vector<Eigen::Matrix4f> total_transform_vec(pairs.size()*n_combi2);
    Eigen::Matrix4f good_transform = Eigen::Matrix4f::Identity();

    #pragma omp parallel for schedule(dynamic) firstprivate(N_hist_axis, lim, clusters1, clusters2, pointNormals_src_init, pointNormals_src, pointNormals_tgt_init, pointNormals_tgt, axis, cloud_src, cloud_tgt ) shared( LCP_vec, total_transform_vec )

    for (int w=0; w<pairs.size(); w++)
    {
        int q=pairs[w].first;
        int p=pairs[w].second;

        vector<vector<double>> walls1(2);

        walls1[0]=clusters1[q].mode;
        walls1[1]=clusters1[p].mode;
        ///initialize cloud_src cloud_tgt and phi--------------------------------------------------------------------------------------------------------------------------------------
        for (int r=0; r<clusters2.size(); r++)
        {
            for (int s=0; s<clusters2.size(); s++)
            {
                if(s!=r)
                {
                int LCP=0;
                vector<vector<double>> walls2(2);
                walls2[0]=clusters2[r].mode;
                walls2[1]=clusters2[s].mode;

                pcl::PointCloud<pcl::PointNormal> source;
                pcl::PointCloud<pcl::PointNormal> target;

                pcl::copyPointCloud(pointNormals_src_init,source);
                pcl::copyPointCloud(pointNormals_tgt_init,target);

             ///rotate cloud in the accurate way with rotation information--------------------------------------------------------------------------------------------------------------------------------------

                Eigen::Matrix4f transformation= Eigen::Matrix4f::Identity();;
                Eigen::Matrix4f rotation_transform = Eigen::Matrix4f::Identity();

                pcl::registration::TransformationEstimationSVD<pcl_point, pcl_point> estimator;
                pcl::PointCloud<pcl_point>::Ptr cloud_clus1(new pcl::PointCloud<pcl_point>);
                pcl::PointCloud<pcl_point>::Ptr cloud_clus2(new pcl::PointCloud<pcl_point>);
                cloud_clus1->width    = 3;
                cloud_clus1->height   = 1;
                cloud_clus1->is_dense = false;
                cloud_clus1->points.resize (cloud_clus1->width * cloud_clus1->height);
                *cloud_clus2=*cloud_clus1;

                for (int i=0; i<2; i++)
                {
                    cloud_clus1->points[i].x=walls1[i][0];
                    cloud_clus1->points[i].y=walls1[i][1];
                    cloud_clus1->points[i].z=walls1[i][2];
                    cloud_clus2->points[i].x=walls2[i][0];
                    cloud_clus2->points[i].y=walls2[i][1];
                    cloud_clus2->points[i].z=walls2[i][2];
                }

                cloud_clus1->points[2].x=0;
                cloud_clus1->points[2].y=0;
                cloud_clus1->points[2].z=0;
                cloud_clus2->points[2].x=0;
                cloud_clus2->points[2].y=0;
                cloud_clus2->points[2].z=0;


                estimator.estimateRigidTransformation(*cloud_clus1, *cloud_clus2, transformation);// ie transformPointCloud(*cloudin, *cloudout, transformation);
                rotation_transform=transformation;
                rotation_transform(0,3)=0;
                rotation_transform(1,3)=0;
                rotation_transform(2,3)=0;

//                std::cout<<"   rotation : "<<std::endl<<rotation_transform<<std::endl<<std::endl;
                pcl::transformPointCloudWithNormals (source, source, rotation_transform);

                ///get translation with histograms correlation

                Eigen::Matrix4f translation_transform = Eigen::Matrix4f::Zero();
                pcl::PointCloud<pcl::PointNormal>::Ptr source_ptr(new pcl::PointCloud<pcl::PointNormal>);
                pcl::PointCloud<pcl::PointNormal>::Ptr target_ptr(new pcl::PointCloud<pcl::PointNormal>);
                *source_ptr=source;
                *target_ptr=target;
                get_translation(source_ptr, target_ptr, lim, axis, N_hist_axis, &translation_transform) ;

                Eigen::Matrix4f total_transform = Eigen::Matrix4f::Zero();
                total_transform=rotation_transform+translation_transform;
//                std::cout<<"total transformation : "<<std::endl<<total_transform<<std::endl<<std::endl;

//                std::cout<<"----------------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl<<std::endl;

                ///gcompute LCP for this transformation

//                pcl::PointCloud<pcl_point> src;
//                pcl::PointCloud<pcl_point> tgt;

//                pcl::copyPointCloud(*cloud_src, src);
//                pcl::copyPointCloud(*cloud_tgt,tgt);

                get_LCP(*cloud_src, *cloud_tgt, &total_transform, &LCP);

                LCP_vec[(s+1)*r*(w+1)]=LCP;
                total_transform_vec[(s+1)*r*(w+1)]=total_transform;

                }
            }
         }

    }

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

        ///save best transformation in build/transformations
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
        sstm<<"transformations/"<<file_name1<<"_"<<file_name2<<".txt";
        std::string file_name_tot = sstm.str();
        ofstream file (file_name_tot);
        file<<good_transform;
        file.close();

        t_tot=clock()-t_tot;
        std::cout<<"total time to get transform :" <<((float)t_tot)/CLOCKS_PER_SEC<<" seconds"<<std::endl<<std::endl;

        std::cout<<"best transformation :"<<std::endl<<good_transform<<std::endl<<std::endl;


}


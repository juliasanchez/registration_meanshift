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
#include <pcl/ml/kmeans.h>

#include "cloud.h"
#include "pre_process.h"
#include "save_vector.h"
#include "save_axis.h"
#include "get_translation.h"
#include "get_rotation.h"
#include "get_LCP.h"
#include "save_normals.h"
#include "save_clusters.h"
#include "pcn2pc.h"
#include "get_axis2.h"
#include "icp.h"
#include "pc2vecvec.h"
#include "density_filter.h"
#include "filter_clusters.h"


bool comp_clus(Cluster, Cluster);
std::string extract_object_from_filename(std::string);


typedef pcl::PointNormal pcl_point;

int main(int argc, char *argv[])
{

    if(argc!=11)
    {
        std::cout<<"classic usage :"<<std::endl;
        std::cout<<"file1  file2 sample_coeff radius_for_normals density_radius_to_filter_normals number_points_meanshifts bin_width_for_translation uniform_filter_parameter_for_LCP output_folder max_distance_points"<<std::endl<<std::endl;
    }
    ///preprocess clouds--------------------------------------------------------------------------------------------------------------

    std::string output=argv[9];

    auto t_tot1 = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt(new pcl::PointCloud<pcl_point>);

    float sample =atof(argv[3]);
    float normal_radius=atof(argv[4]);
    double reso1;
    double reso2;
    float far = atof(argv[10]);
    cloud<pcl_point> cld_manager_src;
    cloud<pcl_point> cld_manager_tgt;

    cld_manager_src.setInputCloud(cloud_src);
    cld_manager_tgt.setInputCloud(cloud_tgt);

    pre_process(argv[1],sample, normal_radius, far, &cld_manager_src, &reso1);
    pre_process(argv[2],sample, normal_radius, far, &cld_manager_tgt, &reso2);

    pcl::PointCloud<pcl_point>::Ptr cloud_src_cpy(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt_cpy(new pcl::PointCloud<pcl_point>);
    *cloud_src_cpy = *cloud_src;
    *cloud_tgt_cpy = *cloud_tgt;

    for (int i = 0; i<cloud_src_cpy->size(); ++i)
    {
        cloud_src_cpy->points[i].normal_x = abs(cloud_src_cpy->points[i].normal_x);
        cloud_src_cpy->points[i].normal_y = abs(cloud_src_cpy->points[i].normal_y);
        cloud_src_cpy->points[i].normal_z = abs(cloud_src_cpy->points[i].normal_z);
    }

    for (int i = 0; i<cloud_tgt_cpy->size(); ++i)
    {
        cloud_tgt_cpy->points[i].normal_x = abs(cloud_tgt_cpy->points[i].normal_x);
        cloud_tgt_cpy->points[i].normal_y = abs(cloud_tgt_cpy->points[i].normal_y);
        cloud_tgt_cpy->points[i].normal_z = abs(cloud_tgt_cpy->points[i].normal_z);
    }

    pcl::io::savePCDFileASCII ("preprocess_src.csv", *cloud_src_cpy);
    pcl::io::savePCDFileASCII ("preprocess_tgt.csv", *cloud_tgt_cpy);

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt(new pcl::PointCloud<pcl::PointNormal>);

    *pointNormals_src = *cloud_src;
    *pointNormals_tgt = *cloud_tgt;

    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_src, *pointNormals_src, indices);
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_tgt, *pointNormals_tgt, indices);

    std::cout<< "source: points number after preprocessing : "<<pointNormals_src->size()<<std::endl;
    std::cout<< "target: points number after preprocessing : "<<pointNormals_tgt->size()<<std::endl<<std::endl;

    auto t_tot1_no_preproc= std::chrono::high_resolution_clock::now();

    ///GET CLUSTERS oF NORMALS--------------------------------------------------------------------------------------------------------------------------------------------

    //1_ Create a pointcloud representing normals as points on a sphere.................................................................................

    pcl::PointCloud<pcl::PointXYZ>::Ptr normals1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr normals2(new pcl::PointCloud<pcl::PointXYZ>);
    pcn2pc(pointNormals_src, normals1);
    pcn2pc(pointNormals_tgt, normals2);

    //2_ Filter normals to enhance clusters   /!\ HEAVY STEP ..........................................................................................

    auto t_filter_1 = std::chrono::high_resolution_clock::now();

    std::cout<<"cloud source: normal points kept for filtering by density: "<<normals1->points.size()<<std::endl;
    std::cout<<"cloud target: normal points kept for filtering by density : "<<normals2->points.size()<<std::endl<<std::endl;

//    pcl::io::savePCDFileASCII ("normals1_before.csv", *normals1);
//    pcl::io::savePCDFileASCII ("normals2_before.csv", *normals2);

    float radius =atof(argv[5]);
    int keep =atoi(argv[6]);

    std::vector<std::vector<float>> modes1; // modes extracted from a simple mean in raw clusters found by density they may be not as good as the ones found by meanshift
    std::vector<std::vector<float>> modes2;

    density_filter(normals1, radius, keep, modes1);
    density_filter(normals2, radius, keep, modes2); // radius = neighborhood size to define density // keep = normals number to compute meanshift

//    pcl::io::savePCDFileASCII ("normals1_after.csv", *normals1);
//    pcl::io::savePCDFileASCII ("normals2_after.csv", *normals2);

    auto t_filter_2 = std::chrono::high_resolution_clock::now();
    std::cout<<"total time to filter normals :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_filter_2-t_filter_1).count()<<" milliseconds"<<std::endl<<std::endl;

    //3_ Find clusters with meanshift computation......................................................................................................

    auto t_meanshifts1 = std::chrono::high_resolution_clock::now();

    std::cout<<"cloud source: normal points kept for meanshift : "<<normals1->points.size()<<std::endl;
    std::cout<<"cloud target: normal points kept for meanshift : "<<normals2->points.size()<<std::endl<<std::endl;

    std::vector< vector<double> > vec_normals_src;
    std::vector< vector<double> > vec_normals_tgt;

    pc2vecvec(normals1, vec_normals_src);
    pc2vecvec(normals2, vec_normals_tgt);

    double kernel_bandwidth = 0.1;
    MeanShift *msp = new MeanShift();
    vector<Cluster> clusters1 = msp->cluster(vec_normals_src, kernel_bandwidth);
    vector<Cluster> clusters2 = msp->cluster(vec_normals_tgt, kernel_bandwidth);

    std::cout<<"number of clusters found in source : "<<clusters1.size()<<std::endl;
    std::cout<<"number of clusters found in target : "<<clusters2.size()<<std::endl<<std::endl;

    auto t_meanshifts2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration_cast<std::chrono::milliseconds>(t_meanshifts2-t_meanshifts1).count();
    std::cout<<"total time to get clusters with meanshift :" << std::chrono::duration_cast<std::chrono::milliseconds>(t_meanshifts2-t_meanshifts1).count()<<" milliseconds"<<std::endl<<std::endl;

    //4_ Filter clusters to remove too spacely large (corresponding to curved sections)...............................................................

    filter_clusters(clusters1);
    filter_clusters(clusters2);

    //5_ Sort clusters and keep only the 6th most important...........................................................................................

    sort(clusters1.begin(), clusters1.end(),comp_clus);
    sort(clusters2.begin(), clusters2.end(),comp_clus);

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

    //6_ Keep all mode normals in a vector............................................................................................................

    std::vector<std::vector<float>> vec_normals1(clusters1.size(), std::vector<float>(3));
    std::vector<std::vector<float>> vec_normals2(clusters2.size(), std::vector<float>(3));

    for(int clus = 0; clus < clusters1.size(); clus++)
    {
        vec_normals1[clus][0]=clusters1[clus].mode[0];
        vec_normals1[clus][1]=clusters1[clus].mode[1];
        vec_normals1[clus][2]=clusters1[clus].mode[2];

    }

    for(int clus = 0; clus < clusters2.size(); clus++)
    {
        vec_normals2[clus][0]=clusters2[clus].mode[0];
        vec_normals2[clus][1]=clusters2[clus].mode[1];
        vec_normals2[clus][2]=clusters2[clus].mode[2];
    }

    //7_ Save mode normals in files "clusterk_modei.csv".............................................................................................

    save_clusters(vec_normals1, "cluster1_mode");
    save_clusters(vec_normals2, "cluster2_mode");


    ///Transformations computation-------------------------------------------------------------------------------------------------------------------------------

    //1_ Build pairs with non parallel normals

    std::vector< pair<int,int> > pairs1;
    for (int q=0; q<vec_normals1.size()-1; q++)
    {
        for (int p=q+1; p<vec_normals1.size(); p++)
        {
          if( abs(vec_normals1[p][0]*vec_normals1[q][0] + vec_normals1[p][1]*vec_normals1[q][1] + vec_normals1[p][2]*vec_normals1[q][2])<0.9)
            pairs1.push_back(make_pair(q,p));
        }
    }

    std::vector< pair<int,int> > pairs2;
    for (int q=0; q<vec_normals2.size(); q++)
    {
        for (int p=0; p<vec_normals2.size(); p++)
        {
          if( abs(vec_normals2[p][0]*vec_normals2[q][0] + vec_normals2[p][1]*vec_normals2[q][1] + vec_normals2[p][2]*vec_normals2[q][2])<0.9 && p!=q)
            pairs2.push_back(make_pair(q,p));
        }
    }

    std::cout<<"number of pairs of cloud 1 : "<<pairs1.size()<<std::endl;
    std::cout<<"number of pairs of cloud 2 : "<<pairs2.size()<<std::endl<<std::endl;
    std::cout<<"number of trials : "<<pairs1.size()*pairs2.size()<<std::endl<<std::endl;

    //2_ Variables and parameters initialization

    float bin_width=atof(argv[7]); // for translation
    float lim = 0.99;
    std::vector<int> LCP_vec(pairs1.size()*vec_normals2.size()*vec_normals2.size());
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> total_transform_vec(pairs1.size()*vec_normals2.size()*vec_normals2.size());
    std::vector< std::vector <std::vector<float> > > total_axis(pairs1.size()*vec_normals2.size()*vec_normals2.size(), std::vector <std::vector<float> >(3, std::vector<float>(3)));
    Eigen::Matrix4f good_transform = Eigen::Matrix4f::Identity();

    //3_ Sample clouds to speed up transform and LCP calculation........................................................................................

    float LCP_samp = atof(argv[8]);
    cld_manager_src.sample(LCP_samp);

    pcl::KdTreeFLANN<pcl_point>::Ptr tree_tgt (new pcl::KdTreeFLANN<pcl_point>);
    tree_tgt->setInputCloud(cloud_tgt);

    std::cout<<"points number for LCP calculation :"<< cloud_src->points.size()<<"  "<<cloud_tgt->points.size()<<std::endl<<std::endl;

    auto t_loop1= std::chrono::high_resolution_clock::now();

    //4_ Main loop to compute each transformation........................................................................................

    #pragma omp parallel for schedule(dynamic) num_threads(omp_get_max_threads()) shared( pairs1, bin_width, lim, vec_normals1, vec_normals2, pointNormals_src, pointNormals_tgt, cloud_src, tree_tgt, LCP_vec, total_transform_vec, total_axis )

    for (int w=0; w<pairs1.size(); w++)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr transformed_source(new pcl::PointCloud<pcl::PointNormal>);
        int q=pairs1[w].first;
        int p=pairs1[w].second;

        vector<vector<float>> walls1(2);
        vector<vector<float>> walls2(2);

        walls1[0]=vec_normals1[q];
        walls1[1]=vec_normals1[p];

        float dot=walls1[0][0]*walls1[1][0]+walls1[0][1]*walls1[1][1]+walls1[0][2]*walls1[1][2]; // allows to check if the angle between pairs is the same
        float al1=acos(dot);

        for (int m=0; m<pairs2.size(); m++)
        {      
            int r=pairs2[m].first;
            int s=pairs2[m].second;         
            walls2[0]=vec_normals2[r];
            walls2[1]=vec_normals2[s];
            dot=walls2[0][0]*walls2[1][0]+walls2[0][1]*walls2[1][1]+walls2[0][2]*walls2[1][2];
            float al2=acos(dot);


            //4_1_ Get 2 main walls and their normal to perform translation search. These normals represent translation axis...............

            std::vector<std::vector<float>> axis(3, std::vector<float>(3) );

            for(int i=0; i<3; i++)
            {
                axis[0][i]=walls2[0][i];
                axis[1][i]=walls2[1][i];
            }

            if (abs(al1-al2)<0.05)  //Checking if angles are the same
            {
                Eigen::Matrix4f rotation_transform = Eigen::Matrix4f::Identity();
                get_rotation(walls1, walls2, &rotation_transform);
                pcl::transformPointCloudWithNormals (*pointNormals_src, *transformed_source, rotation_transform);

                std::vector<std::vector<float>> vec_normals1cpy (vec_normals1.size(), std::vector<float>(3));

                for (int k=0; k<vec_normals1cpy.size(); k++)
                {
                    vec_normals1cpy[k][0]=vec_normals1[k][0]*rotation_transform(0,0)+vec_normals1[k][1]*rotation_transform(0,1)+vec_normals1[k][2]*rotation_transform(0,2);
                    vec_normals1cpy[k][1]=vec_normals1[k][0]*rotation_transform(1,0)+vec_normals1[k][1]*rotation_transform(1,1)+vec_normals1[k][2]*rotation_transform(1,2);
                    vec_normals1cpy[k][2]=vec_normals1[k][0]*rotation_transform(2,0)+vec_normals1[k][1]*rotation_transform(2,1)+vec_normals1[k][2]*rotation_transform(2,2);
                }
                get_axis2(vec_normals1cpy, vec_normals2, axis); // Get a third axis to perform translation

//                save_axis(axis[0], "axisx.txt");
//                save_axis(axis[1], "axisy.txt");
//                save_axis(axis[2], "axisz.txt");

                //4_2_ Get translation with histograms correlation........................................................................

                Eigen::Matrix4f translation_transform = Eigen::Matrix4f::Zero();
                bool sat=true; // we want to saturate the bins to avoid very dense walls to corrupt correlation computation
                get_translation(transformed_source, pointNormals_tgt, lim, axis, bin_width, sat, &translation_transform) ;

                ///compute LCP for this transformation

                int lim_for=vec_normals2.size();
                int idx = w*lim_for*lim_for+r*lim_for+s;

                total_transform_vec[idx] = rotation_transform+translation_transform;
                total_axis[idx]=axis;
                get_LCP(cloud_src, tree_tgt, sample+bin_width, &total_transform_vec[idx], &LCP_vec[idx]);
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

    int index2=0;
    temp=0;
    for (int i=0; i<LCP_vec.size(); i++)
    {
        if(temp<LCP_vec[i] && abs(total_transform_vec[i](0,0)-total_transform_vec[index](0,0))> 0.05)
        {
            temp=LCP_vec[i];
            index2=i;
        }
    }

    if(LCP_vec[index]==0)
    {
        std::cout<<"best LCP = 0 --> aborting "<<std::endl<<std::endl;
        good_transform = Eigen::Matrix4f::Identity();
    }
    else
    {

        good_transform=total_transform_vec[index];


        pcl::transformPointCloudWithNormals (*pointNormals_src, *pointNormals_src, good_transform);
        Eigen::Matrix4f translation_transform = Eigen::Matrix4f::Zero();
        bool sat=false;

//        save_axis(total_axis[index][0], "axis1.txt");
//        save_axis(total_axis[index][1], "axis2.txt");
//        save_axis(total_axis[index][2], "axis3.txt");

        std::cout<<"coarse transform :"<<std::endl<<good_transform<<std::endl<<LCP_vec[index]<<std::endl<<std::endl;

        if(abs(LCP_vec[index]-LCP_vec[index2])<0.1*LCP_vec[index])
        {
            std::cout<<"2nd choice :"<<std::endl<<total_transform_vec[index2]<<std::endl<<LCP_vec[index2]<<std::endl<<std::endl;
        }
        get_translation(pointNormals_src, pointNormals_tgt, lim, total_axis[index], bin_width, sat, &translation_transform) ;

        good_transform += translation_transform;
    }

    auto t_meth = std::chrono::high_resolution_clock::now();
    std::cout<<"time to get transform with our method :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_meth-t_tot1_no_preproc).count()<<" milliseconds"<<std::endl<<std::endl;

    std::cout<<"best transformation :"<<std::endl<<good_transform<<std::endl<<std::endl;

    ///save best transformation in build/results/transformations and the time

    std::stringstream sstm;
    sstm.str("");
    sstm<<"mkdir "<<output;
    std::string command = sstm.str();
    const char* c=command.c_str();
    system(c);

    std::string object1 = extract_object_from_filename(argv[1]);
    std::string object2 = extract_object_from_filename(argv[2]);

    sstm.str("");
    sstm<<output<<"/"<<object1<<"_"<<object2<<".txt";
    std::string file_name_tot = sstm.str();
    ofstream file (file_name_tot);
    file<<good_transform;
    file<<"\n";
    file<<std::chrono::duration_cast<std::chrono::milliseconds>(t_meth-t_tot1_no_preproc).count();
    file<<"\n";
    file<<std::chrono::duration_cast<std::chrono::milliseconds>(t_meth-t_tot1).count();
    file<<"\n";
    file<<reso1<<" "<<reso2;
    file.close();

//        file.open("/home/julia/Desktop/my_programs/hist_registration/build-hist_registration/results/Times", std::ofstream::out | std::ofstream::app);
//        file<<"\n"<<"\n"<<"\n"
//            <<"file1: "<<argv[1]<<" and "
//            <<"file2: "<<argv[2]<<std::endl
//            <<"sample_coeff: "<<argv[3]<<std::endl
//            <<"normal_filter_parameter: "<<argv[5]<<std::endl
//            <<"bin_width_for_translation: "<<argv[6]<<std::endl
//            <<"TIME: "<<std::chrono::duration_cast<std::chrono::milliseconds>(t_tot2-t_tot1).count();
//        file.close();

}





//TOOL FUNCTIONS


bool comp_clus(Cluster clus1, Cluster clus2 )
{
    if ( clus1.original_points.size()>clus2.original_points.size() )
    {
        return true;
    }
    return false;
}

std::string extract_object_from_filename(std::string file_name)
{
    size_t lastindex_point = file_name.find_last_of(".");
    size_t lastindex_slash = file_name.find_last_of("/");
    if (lastindex_slash==std::string::npos)
    {
       lastindex_slash = 0;
    }

    return file_name.substr(lastindex_slash+1, lastindex_point-(lastindex_slash+1));
}


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
#include "get_axis.h"
#include "icp.h"
#include "pc2vecvec.h"
#include "density_filter.h"
#include "MeanShift.h"

std::string extract_object_from_filename(std::string);

typedef pcl::PointNormal pcl_point;

int main(int argc, char *argv[])
{
    if(argc!=11)
    {
        std::cout<<"classic usage :"<<std::endl;
        std::cout<<"file1  file2 sample_coeff radius_for_normals angle_uncertainty_on_planes_normals(degrees) points_to_keep_for_meanshift  bin_width_for_translation uniform_filter_parameter_for_LCP output_folder max_distance_points"<<std::endl<<std::endl;
//                                        0.06     0.1                       1             	1500                                0.01                    0.06                            test             200
    }
    ///preprocess clouds--------------------------------------------------------------------------------------------------------------

    std::string output=argv[9];

    auto t_tot1 = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt(new pcl::PointCloud<pcl::PointNormal>);

    float sample =atof(argv[3]);
    float normal_radius=atof(argv[4]);
    double reso1;
    double reso2;
    float far = atof(argv[10]);
    cloud cld_manager_src;
    cloud cld_manager_tgt;

    cld_manager_src.setInputCloud(pointNormals_src);
    cld_manager_tgt.setInputCloud(pointNormals_tgt);

    pre_process(argv[1],sample, normal_radius, far, &cld_manager_src, &reso1);
    pre_process(argv[2],sample, normal_radius, far, &cld_manager_tgt, &reso2);

    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_src, *pointNormals_src, indices);
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_tgt, *pointNormals_tgt, indices);

    std::string object1 = extract_object_from_filename(argv[1]);
    std::stringstream stm;
    stm<<object1<<".pcd";
    pcl::io::savePCDFileASCII (stm.str(), *pointNormals_src);
    pcl::io::savePCDFileASCII ("source.csv", *pointNormals_src);

    std::string object2 = extract_object_from_filename(argv[2]);
    stm.str("");
    stm<<object2<<".pcd";
    pcl::io::savePCDFileASCII (stm.str(), *pointNormals_tgt);
    pcl::io::savePCDFileASCII ("target.csv", *pointNormals_tgt);

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

    pcl::io::savePCDFileASCII ("normals1_before.csv", *normals1);
    pcl::io::savePCDFileASCII ("normals2_before.csv", *normals2);

    system("rm large_**");

    int keep = atoi(argv[6]);
    float angle_incertainty = atof(argv[5])*M_PI/180;
    float large_angle = 15*M_PI/180;

    density_filter(normals1, angle_incertainty, large_angle, keep);
    density_filter(normals2, angle_incertainty, large_angle, keep); // angle_incertainty = uncertainty on normal estimation on plane

    pcl::io::savePCDFileASCII ("normals1_after.csv", *normals1);
    pcl::io::savePCDFileASCII ("normals2_after.csv", *normals2);

    auto t_filter_2 = std::chrono::high_resolution_clock::now();
    std::cout<<"total time to filter normals :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_filter_2-t_filter_1).count()<<" milliseconds"<<std::endl<<std::endl;

    //3_ Find clusters with meanshift computation......................................................................................................

    auto t_meanshifts1 = std::chrono::high_resolution_clock::now();

    std::cout<<"cloud source: normal points kept for meanshift : "<<normals1->points.size()<<std::endl;
    std::cout<<"cloud target: normal points kept for meanshift : "<<normals2->points.size()<<std::endl<<std::endl;

    std::vector< vector<double> > vec_normals_src;
    std::vector< vector<double> > vec_normals_tgt;

    //convert to enter meanshift algo
    pc2vecvec(normals1, vec_normals_src);
    pc2vecvec(normals2, vec_normals_tgt);

    double density_radius = sqrt(2*(1-cos(angle_incertainty)));
    double kernel_bandwidth = 2*density_radius;
    MeanShift *msp = new MeanShift();
    std::vector<Cluster> clusters1 = msp->cluster(vec_normals_src, kernel_bandwidth);
    std::vector<Cluster> clusters2 = msp->cluster(vec_normals_tgt, kernel_bandwidth);

    std::cout<<std::endl<<"number of clusters found in source : "<<clusters1.size()<<std::endl;
    std::cout<<"number of clusters found in target : "<<clusters2.size()<<std::endl<<std::endl;

    auto t_meanshifts2 = std::chrono::high_resolution_clock::now();
    std::cout<<"total time to get clusters with meanshift :" << std::chrono::duration_cast<std::chrono::milliseconds>(t_meanshifts2-t_meanshifts1).count()<<" milliseconds"<<std::endl<<std::endl;

    //6_ Keep all mode normals in a vector............................................................................................................

    std::vector<Eigen::Vector3f> vec_normals1(clusters1.size());  // modes extracted from a simple mean in raw clusters found by density they may be not as good as the ones found by meanshift
    std::vector<Eigen::Vector3f> vec_normals2(clusters2.size());
    for(int clus = 0; clus < clusters1.size(); clus++)
    {
        vec_normals1[clus](0)=clusters1[clus].mode[0];
        vec_normals1[clus](1)=clusters1[clus].mode[1];
        vec_normals1[clus](2)=clusters1[clus].mode[2];
        vec_normals1[clus] /= vec_normals1[clus].norm();
    }

    for(int clus = 0; clus < clusters2.size(); clus++)
    {
        vec_normals2[clus][0]=clusters2[clus].mode[0];
        vec_normals2[clus][1]=clusters2[clus].mode[1];
        vec_normals2[clus][2]=clusters2[clus].mode[2];
        vec_normals2[clus] /= vec_normals2[clus].norm();
    }

    //3_ Save mode normals in files "clusterk_modei.csv".............................................................................................

    system("rm cluster*");
    save_clusters(vec_normals1, "cluster1_mode");
    save_clusters(vec_normals2, "cluster2_mode");


    ///Transformations computation-------------------------------------------------------------------------------------------------------------------------------

    //1_ Build pairs with non parallel normals

    std::vector< pair<int,int> > pairs1;
    for (int q=0; q<vec_normals1.size()-1; q++)
    {
        for (int p=q+1; p<vec_normals1.size(); p++)
        {
            if( abs(vec_normals1[p].dot(vec_normals1[q]) )<0.9)
                pairs1.push_back(make_pair(q,p));
        }
    }

    std::vector< pair<int,int> > pairs2;
    for (int q=0; q<vec_normals2.size(); q++)
    {
        for (int p=0; p<vec_normals2.size(); p++)
        {
            if( abs(vec_normals2[p].dot(vec_normals2[q]) )<0.9)
                pairs2.push_back(make_pair(q,p));
        }
    }

    std::cout<<"number of pairs of cloud 1 : "<<pairs1.size()<<std::endl;
    std::cout<<"number of pairs of cloud 2 : "<<pairs2.size()<<std::endl<<std::endl;
    std::cout<<"number of trials : "<<pairs1.size()*pairs2.size()<<std::endl<<std::endl;

    //2_ Variables and parameters initialization

    float bin_width=atof(argv[7]); // for translation
    float lim = 0.99;
    std::vector<float> LCP_vec(pairs1.size()*vec_normals2.size()*vec_normals2.size());
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> total_transform_vec(pairs1.size()*vec_normals2.size()*vec_normals2.size());
    std::vector< std::vector <Eigen::Vector3f> > total_axis(pairs1.size()*vec_normals2.size()*vec_normals2.size(), std::vector<Eigen::Vector3f>(3));
    Eigen::Matrix4f good_transform = Eigen::Matrix4f::Identity();

    //3_ Sample clouds to speed up transform and LCP calculation........................................................................................

    float LCP_samp = atof(argv[8]);
    cld_manager_src.sample(LCP_samp);

    pcl::KdTreeFLANN<pcl::PointNormal>::Ptr tree_tgt (new pcl::KdTreeFLANN<pcl::PointNormal>);
    tree_tgt->setInputCloud(pointNormals_tgt);

    std::cout<<"points number for LCP calculation :"<< pointNormals_src->points.size()<<"  "<<pointNormals_tgt->points.size()<<std::endl<<std::endl;

    auto t_loop1= std::chrono::high_resolution_clock::now();

    //4_ Main loop to compute each transformation........................................................................................

    #pragma omp parallel for schedule(dynamic) num_threads(omp_get_max_threads()) shared( pairs1, bin_width, lim, vec_normals1, vec_normals2, pointNormals_src, pointNormals_tgt, tree_tgt, LCP_vec, total_transform_vec, total_axis )
    for (int w=0; w<pairs1.size(); w++)
    {
        int q=pairs1[w].first;
        int p=pairs1[w].second;
        float al1=acos(vec_normals1[q].dot(vec_normals1[p]));// allows to check if the angle between pairs is the same

        if(al1*180/M_PI > 10 && al1*180/M_PI < 170)
        {
            for (int m=0; m<pairs2.size(); m++)
            {
                int r=pairs2[m].first;
                int s=pairs2[m].second;
                float al2=acos(vec_normals2[r].dot(vec_normals2[s]));

                //4_1_ Get 2 main walls and their normal to perform translation search. These normals represent translation axis...............

                std::vector<Eigen::Vector3f> axis(3);
                axis[0]=vec_normals2[r];
                axis[1]=vec_normals2[s];

                if (abs(al1-al2)*180/M_PI<3)  //Checking if angles are the same (3° of difference max between normals)
                {
                    pcl::PointCloud<pcl::PointNormal>::Ptr transformed_source(new pcl::PointCloud<pcl::PointNormal>);
                    Eigen::Matrix4f rot_transfo = Eigen::Matrix4f::Identity();
                    std::pair<Eigen::Vector3f,Eigen::Vector3f> walls1 = std::pair<Eigen::Vector3f,Eigen::Vector3f>(vec_normals1[q], vec_normals1[p]);
                    std::pair<Eigen::Vector3f,Eigen::Vector3f> walls2 = std::pair<Eigen::Vector3f,Eigen::Vector3f>(vec_normals2[r], vec_normals2[s]);
                    get_rotation(walls1, walls2, &rot_transfo);
                    pcl::transformPointCloudWithNormals (*pointNormals_src, *transformed_source, rot_transfo);

                    std::vector<Eigen::Vector3f> vec_normals1cpy (vec_normals1.size());

                    for (int k=0; k<vec_normals1cpy.size(); k++)
                        vec_normals1cpy[k]=rot_transfo.block<3,3>(0,0) * vec_normals1[k];

                    get_axis(vec_normals1cpy, vec_normals2, axis); // Get a third axis to perform translation

                    //4_2_ Get translation with histograms correlation........................................................................

                    Eigen::Matrix4f translation_transform = Eigen::Matrix4f::Zero();
                    bool sat=true; // we want to saturate the bins to avoid very dense walls to corrupt correlation computation
                    get_translation(transformed_source, pointNormals_tgt, lim, axis, bin_width, sat, &translation_transform) ;

                    ///compute LCP for this transformation

                    int lim_for = vec_normals2.size();
                    int idx = w*lim_for*lim_for+r*lim_for+s;

                    total_transform_vec[idx] = rot_transfo+translation_transform;
                    total_axis[idx]=axis;
                    get_LCP(pointNormals_src, tree_tgt, bin_width*sqrt(3), &total_transform_vec[idx], &LCP_vec[idx]);
                }
              }
       }
    }

    auto t_loop2 = std::chrono::high_resolution_clock::now();
    std::cout<<"time for the loop :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_loop2-t_loop1).count()<<" milliseconds"<<std::endl<<std::endl;

    ///get best transform using LCP value

    int index=0;
    int index2=0;
    float temp=0;
    for (int i=0; i<LCP_vec.size(); i++)
    {
        if(temp<LCP_vec[i])
        {
            temp=LCP_vec[i];
            index2 = index;
            index=i;
        }
    }

    std::cout<<"second LCP chosen : "<<LCP_vec[index2]<<std::endl;
    std::cout<<"LCP chosen : "<<LCP_vec[index]<<std::endl<<std::endl;

    if(LCP_vec[index]==0)
    {
        std::cout<<"best LCP = 0 --> aborting "<<std::endl<<std::endl;
        good_transform = Eigen::Matrix4f::Identity();
    }
    else
    {
        bool sat=false;

        pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src1(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src2(new pcl::PointCloud<pcl::PointNormal>);

        Eigen::Matrix4f translation_transform1 = Eigen::Matrix4f::Zero();
        Eigen::Matrix4f translation_transform2 = Eigen::Matrix4f::Zero();

        pcl::transformPointCloudWithNormals (*pointNormals_src, *pointNormals_src1, total_transform_vec[index]);
        get_translation(pointNormals_src1, pointNormals_tgt, lim, total_axis[index], bin_width, sat, &translation_transform1) ;
        total_transform_vec[index] += translation_transform1;

        get_LCP(pointNormals_src, tree_tgt, reso1, &total_transform_vec[index], &LCP_vec[index]);
        std::cout<<"1st best transform :"<<std::endl<<total_transform_vec[index]<<std::endl<<std::endl;
        std::cout<<"1st LCP after refinement: "<<LCP_vec[index]<<std::endl<<std::endl;
        good_transform = total_transform_vec[index];

        if(LCP_vec[index2]>0)
        {
            pcl::transformPointCloudWithNormals (*pointNormals_src, *pointNormals_src2, total_transform_vec[index2]);
            get_translation(pointNormals_src2, pointNormals_tgt, lim, total_axis[index2], bin_width, sat, &translation_transform2) ;
            total_transform_vec[index2] += translation_transform2;
            get_LCP(pointNormals_src, tree_tgt, reso1, &total_transform_vec[index2], &LCP_vec[index2]);
            std::cout<<"2nd best transform  :"<<std::endl<<total_transform_vec[index2]<<std::endl<<std::endl;
            std::cout<<"2nd LCP after refinement: "<<LCP_vec[index2]<<std::endl<<std::endl;
            if(LCP_vec[index2]>LCP_vec[index])
                good_transform = total_transform_vec[index2];
        }
    }

    auto t_meth = std::chrono::high_resolution_clock::now();
    std::cout<<"time to get transform with our method :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_meth-t_tot1_no_preproc).count()<<" milliseconds"<<std::endl<<std::endl;

    std::cout<<"best transformation :"<<std::endl<<good_transform<<std::endl<<std::endl;

   if(LCP_vec[index2]>LCP_vec[index])
   {
        save_axis(total_axis[index][0], "axis1.txt");
        save_axis(total_axis[index][1], "axis2.txt");
        save_axis(total_axis[index][2], "axis3.txt");
   }
   else
   {
       save_axis(total_axis[index2][0], "axis1.txt");
       save_axis(total_axis[index2][1], "axis2.txt");
       save_axis(total_axis[index2][2], "axis3.txt");
   }

    ///save best transformation in build/results/transformations and the time

    std::stringstream sstm;
    sstm.str("");
    sstm<<"mkdir "<<output;
    std::string command = sstm.str();
    const char* c=command.c_str();
    system(c);

    sstm.str("");
    sstm<<output<<"/"<<object1<<"_"<<object2<<"_ssfr.txt";
    std::string file_name_tot = sstm.str();
    std::cout<<"transform file is : "<<file_name_tot<<std::endl;
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

std::string extract_object_from_filename(std::string file_name)
{
    size_t lastindex_point = file_name.find_last_of(".");
    size_t lastindex_slash = file_name.find_last_of("/");
    if (lastindex_slash==std::string::npos)
       lastindex_slash = -1;

    return file_name.substr(lastindex_slash+1, lastindex_point-(lastindex_slash+1));
}


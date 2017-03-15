#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <iterator>
#include "save_normals.h"
#include "save_cluster.h"
#include "MeanShift.h"
#include <time.h>

using namespace std;

vector<vector<double> > load_points(const char *filename) {

    std::ifstream file(filename);

    file.unsetf(std::ios_base::skipws);

       // count the newlines with an algorithm specialized for counting:
    unsigned line_count = std::count(std::istream_iterator<char>(file), std::istream_iterator<char>(),'\n');
    int samp=8;
    vector< vector<double> > data ((int)(line_count/samp)+1 ,vector<double>(3,0.0) );

    file.close();
    file.open(filename);
    int n=0;

    for(int row = 0; row < line_count; row=row+samp)
    {
        std::string line;
        std::getline(file, line);
        if ( !file.good() )
            break;

        std::stringstream iss(line);

        for (int col = 0; col < data[0].size(); ++col)
        {
            std::string val;
            std::getline(iss, val, ' ');
            if ( iss.good() )
            {
                std::stringstream convertor(val);
                convertor >> data[n][col];
            }
            else if (col==data[0].size()-1)
            {
                std::getline(iss, val);
                std::stringstream convertor(val);
                convertor >> data[n][col];
            }
            else
                break;
        }
        n++;
    }
    save_normals(data, "normals_used.csv");
    return data;
}

void print_points(vector<vector<double> > points){
    for(int i=0; i<points.size(); i++){
        for(int dim = 0; dim<points[i].size(); dim++) {
            printf("%f ", points[i][dim]);
        }
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    clock_t t = clock();

    MeanShift *msp = new MeanShift();
    double kernel_bandwidth = 0.2;

    vector<vector<double> > points = load_points("data/normals.csv");
    vector<Cluster> clusters = msp->cluster(points, kernel_bandwidth);

    t = clock()-t;
    std::cout<<"time to get clusters : "<<((float)t)/CLOCKS_PER_SEC<<" seconds with "<<points.size()<<" points"<<std::endl<<std::endl;

    FILE *fp = fopen("result.csv", "w");
    if(!fp){
        perror("Couldn't write result.csv");
        exit(0);
    }

    printf("\n====================\n");
    printf("Found %lu clusters\n", clusters.size());
    printf("====================\n\n");
    for(int cluster = 0; cluster < clusters.size(); cluster++) {
    std::stringstream sstm;
    sstm<<"mode_"<<cluster<<".csv";
    std::string cluster_name = sstm.str();

      save_cluster(clusters[cluster].mode,cluster_name);
      printf("Cluster %i:\n", cluster);
      for(int point = 0; point < clusters[cluster].original_points.size(); point++){
        for(int dim = 0; dim < clusters[cluster].original_points[point].size(); dim++) {
          printf("%f ", clusters[cluster].original_points[point][dim]);
          fprintf(fp, dim?",%f":"%f", clusters[cluster].original_points[point][dim]);
        }
        printf(" -> ");
        for(int dim = 0; dim < clusters[cluster].shifted_points[point].size(); dim++) {
          printf("%f ", clusters[cluster].shifted_points[point][dim]);
        }
        printf("\n");
        fprintf(fp, "\n");
      }
      printf("\n");
    }
    fclose(fp);

    return 0;
}

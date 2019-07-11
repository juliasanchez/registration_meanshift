void norm_hist(std::vector<float>& hist_axis)
{
//      float med=vecMed(hist_axis);

      float moy = 0;
      for(int i = 0; i<hist_axis.size(); ++i)
          moy += hist_axis[i];
      moy /= hist_axis.size();

      float sd = 0;
      for(int i = 0; i<hist_axis.size(); ++i)
          sd += pow(hist_axis[i]-moy,2);
      sd = sqrt(sd/hist_axis.size());

//      std::cout<<"moyenne : "<<moy<<std::endl;
//      std::cout<<"standard deviation : "<<sd<<std::endl;
      int N_fact = 2;

      for (int i = 0; i < hist_axis.size(); ++i)
      {
            if(hist_axis[i]>N_fact*sd)
               hist_axis[i]= moy + N_fact*sd;
      }
}

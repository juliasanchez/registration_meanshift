void get_hist_axis(std::vector<float>& axis_lim, std::set<double>& proj, bool moy, std::vector<float>& hist_axis )
{
    int N_hist_axis=hist_axis.size();
    float delta=(float)(axis_lim[1]-axis_lim[0]) / (float)(N_hist_axis);

    for (auto proj_it = proj.begin(); proj_it != proj.end(); ++proj_it)
    {        
        int n=(int)(  (*proj_it-axis_lim[0]) /delta  );
        hist_axis[n]=hist_axis[n]+1.0;
    }

    for (int n = 0; n < hist_axis.size(); ++n)
        hist_axis[n] /= proj.size ();

//    save_vector (hist_axis, "hist_axis1.csv");

    /// moyennage histogramme

    if(moy)
    {
        int neigh=(int)(0.01/delta); //neighborhood to compute mean value
        neigh = std::min(neigh, (int)(hist_axis.size()/2));
        float cumul=0;
        std::vector<float> hist0(hist_axis.size(),0.0);

        for(int i=0; i<hist_axis.size(); i++)
        {
            int min_idx = std::max(0, i-neigh);
            int max_idx = std::min((int)(hist_axis.size()-1), i+neigh);
            cumul=0;
            int n = 0;
            for (int k = min_idx; k<=max_idx; ++k)
            {
                cumul+=hist_axis[k];
                ++n;
            }
            hist0[i]=cumul/n;
        }

        //init cumul

//        for(int i=0; i<neigh+1; i++)
//        {
//            cumul=0;
//            for(int k=0; k<i+neigh+1; k++)
//                cumul+=hist_axis[k];
//            hist0[i]=cumul/(i+neigh+1);
//        }

//        if((2*neigh+1)<hist_axis.size())
//        {
//            for(int i=neigh+1; i<hist_axis.size()-neigh; i++)
//            {
//               cumul+=hist_axis[i+neigh];
//               cumul-=hist_axis[i-neigh-1];
//               hist0[i]=cumul/(2*neigh+1);
//            }
//        }


//       for(int i=hist_axis.size()-neigh; i<hist_axis.size(); i++)
//       {
//           cumul=0;
//           for(int k=i-neigh; k<hist_axis.size(); k++)
//               cumul+=hist_axis[k];
//           hist0[i]=cumul/(hist_axis.size()-i+neigh);
//       }

       hist_axis=hist0;
//       int alea = rand();
//       std::stringstream stm;
//       std::cout<<alea<<std::endl;
//       stm<<"hist_axis_moy_"<<alea<<".csv";
//       save_vector (hist_axis, stm.str());
    }

    if(proj.size() != 0)
        norm_hist(hist_axis); // saturate bins with too big/dense walls which can corrupt the computation of the correlation. The saturation value is 5* the bins median. (only bins which have more than 10 points)

}

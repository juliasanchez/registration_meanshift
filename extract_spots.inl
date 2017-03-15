void extract_spots(std::vector<std::vector<float>>& hist_axis, std::vector<std::vector<float>>& spots, float thresh )
{
    int N_hist=2*hist_axis.size();
    float delta= 2*M_PI/(float)(N_hist);
//    float thresh=3000/(N_hist*(N_hist/2));

    for (int i = 0; i < (int)(N_hist/2); ++i)
    {
      for (int j = 0; j < N_hist; ++j)
      {
          if(hist_axis[i][j]>thresh)
          {
            std::vector<float> spot ={i*delta+delta/2,j*delta+delta/2};
            spots.push_back(spot);
          }
      }
    }
}

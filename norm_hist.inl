void norm_hist(std::vector<float>& hist_axis)
{
      int N_hist=hist_axis.size();

      float med=vecMed(hist_axis);

      for (int i = 0; i < N_hist; ++i)
      {
            if(hist_axis[i]>5*med)
               hist_axis[i]=5*med;
      }
}

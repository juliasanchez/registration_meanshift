void norm_hist(std::vector<float>& hist_axis)
{
      int N_hist=hist_axis.size();
      for (int i = 0; i < N_hist; ++i)
      {
                if(hist_axis[i]>(1/(float)N_hist)*20)
		{
                   hist_axis[i]=(1/(float)N_hist)*20;
		}
      }
}

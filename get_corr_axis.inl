void get_corr_axis(std::vector<float>& hist1_axisi, std::vector<float>& hist2_axisi, int margin, std::vector<float>& corr_axis, int *translation_axis)
{
    float temp=0;
    int N_hist_axis= hist1_axisi.size();

    if(margin<3 || margin>N_hist_axis-1 )
        margin = N_hist_axis-1;

    corr_axis.resize(2*margin+1);

//    for (int k=0; k<corr_axis.size(); k++)
//    {
//        int lim_inf=std::max(k-margin,0);
//        int lim_sup=std::min(N_hist_axis-(margin+1-k), N_hist_axis-1);

//        for (int m=lim_inf; m<lim_sup+1; m++)
//            corr_axis[k] += hist2_axisi[m]*hist1_axisi[m+margin-k];

//        if (temp<corr_axis[k])
//        {
//            temp=corr_axis[k];
//            *translation_axis=k-margin;
//        }
//    }

    for (int k=0; k<corr_axis.size(); k++)
    {
        int diff = k-margin;
        int Nm;
        if(diff <= 0)
        {
            Nm = N_hist_axis+diff;
            for(int m = 0; m < Nm; ++m)
                corr_axis[k] += hist2_axisi[m]*hist1_axisi[m-diff];
        }
        else
        {
            Nm = N_hist_axis-diff;
            for(int m = 0; m < Nm; ++m)
                corr_axis[k] += hist2_axisi[m+diff]*hist1_axisi[m];
        }

        if (temp<corr_axis[k])
        {
            temp=corr_axis[k];
            *translation_axis=diff;
        }
    }
}

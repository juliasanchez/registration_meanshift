void get_corr_axis(std::vector<float>& hist1_axisi, std::vector<float>& hist2_axisi, std::vector<float>& corr_axis, int *translation_axis)
{
    int p=0;
    float temp=0;
    int N_hist_axis= hist1_axisi.size();
    std::vector<float> hist11_axis(3*N_hist_axis-2, 0.0);
    std::vector<float> hist21_axis(3*N_hist_axis-2, 0.0);

    for (int m=N_hist_axis-1; m<2*N_hist_axis-1; m++)
    {
        hist21_axis[m]=hist2_axisi[p];
        p++;
    }

    for (int k=0; k<2*N_hist_axis-1; k++)
    {
        //move object of delta_axis

        if(k!=0)
        {
           hist11_axis[k-1]=0;
        }
        for (int m=0; m<N_hist_axis; m++)
        {
            hist11_axis[m+k]=hist1_axisi[m];
        }

        //compute correlation

        for (int m=0; m<3*N_hist_axis-2; m++)
        {
            corr_axis[k]=corr_axis[k]+hist11_axis[m]*hist21_axis[m];
        }

    }

    std::vector<float> env;
    int neighbourhood=10;
    envelope(corr_axis, neighbourhood, &env);
    save_vector(corr_axis, "corr_axis_before.csv");

    for (int k=10; k<2*N_hist_axis-1-neighbourhood; k++)
    {
        corr_axis[k]=corr_axis[k]-env[k-neighbourhood];

        if (temp<corr_axis[k])
        {
            temp=corr_axis[k];
            *translation_axis=k;
        }
    }

}

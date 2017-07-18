void get_corr_axis(std::vector<float>& hist1_axisi, std::vector<float>& hist2_axisi, std::vector<float>& corr_axis, int *translation_axis)
{

//    auto t_get_axis1= std::chrono::high_resolution_clock::now();
    float temp=0;
    int N_hist_axis= hist1_axisi.size();

    for (int k=0; k<2*N_hist_axis-1; k++)
    {
        int lim_inf=std::max(k-(N_hist_axis-1),0);
        int lim_sup=std::min(k, N_hist_axis-1);

        for (int m=lim_inf; m<lim_sup+1; m++)
        {
            corr_axis[k]=corr_axis[k]+hist2_axisi[m]*hist1_axisi[m+(N_hist_axis-1)-k];
        }

    }

//    auto t_get_axis2= std::chrono::high_resolution_clock::now();
//    std::cout<<"time to get rotation:" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_get_axis2-t_get_axis1).count()<<" milliseconds"<<std::endl<<std::endl;

    std::vector<float> env;
    int neighbourhood=10;
    envelope(corr_axis, neighbourhood, &env);
//    save_vector(corr_axis, "corr_axis_before.csv");

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

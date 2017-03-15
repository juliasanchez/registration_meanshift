void envelope(std::vector<float> corr, int neigh, std::vector<float>* env)
{
    std::vector<float> tmp(21,0.0);
    std::vector<float> env0(corr.size()-20,0.0);
    for(int i=10; i<corr.size()-neigh; i++)
    {
        for(int k=-neigh; k<neigh+1; k++)
        {
            tmp[k+neigh]=corr[i+k];
        }

        env0[i-neigh]=vecMed(tmp);
    }

    *env=env0;
//    save_vector(envelope0, "envelope.csv");
}

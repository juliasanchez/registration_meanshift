void envelope(std::vector<float>& corr, int neigh, std::vector<float>& env)
{
    std::vector<float> tmp(2*neigh+1,0.0);

    for(int i=neigh; i<corr.size()-neigh; i++)
    {
        for(int k=-neigh; k<neigh+1; k++)
        {
            tmp[k+neigh]=corr[i+k];
        }

        env[i-neigh]=vecMed(tmp);
    }

//    save_vector(envelope0, "envelope.csv");
}

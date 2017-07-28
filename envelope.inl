void envelope(std::vector<float> corr, int neigh, std::vector<float>* env)
{
    std::vector<float> tmp(2*neigh+1,0.0);
    std::vector<float> env0(corr.size()-2*neigh,0.0);

    for(int i=neigh; i<corr.size()-neigh; i++)
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

//Si je veux l'enveloppe sur touuute la corrélation


//{
//    std::vector<float> tmp; //(2*neigh+1,0.0);
//    std::vector<float> env0(corr.size(),0.0);

//    for(int i=0; i<neigh; i++)
//    {
//        tmp.resize(i+neigh+1);
//        for(int k=-i; k<neigh+1; k++)
//        {
//            tmp[k+i]=corr[i+k];
//        }

//        env0[i]=vecMed(tmp);
//    }

//    for(int i=neigh; i<corr.size()-neigh; i++)
//    {
//        for(int k=-neigh; k<neigh+1; k++)
//        {
//            tmp[k+neigh]=corr[i+k];
//        }

//        env0[i]=vecMed(tmp);
//    }

//    for(int i=corr.size()-neigh; i<corr.size(); i++)
//    {
//        tmp.resize(corr.size()-i+neigh+1);
//        for(int k=-neigh; k<corr.size()-i+1; k++)
//        {
//            tmp[k+i]=corr[i+k];
//        }

//        env0[i]=vecMed(tmp);
//    }
//    *env=env0;
//}






// float cumul=0;
// std::vector<float> env0(corr.size()-2*neigh,0.0);

//    for(int k=-neigh; k<neigh+1; k++)
//    {
//        cumul+=corr[k];
//    }

//    env0[0]=cumul/(2*neigh+1);
//    for(int i=neigh; i<corr.size()-neigh; i++)
//    {
//        cumul+=corr[i+neigh];
//        cumul-=corr[i-neigh-1];
//        env0[i-neigh]=cumul/(2*neigh+1);
//    }

//    *env=env0;

//}

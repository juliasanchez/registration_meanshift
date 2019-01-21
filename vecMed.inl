float vecMed(std::vector<float> vec)
{
    if(vec.empty()) return 0;
    else
    {
        std::vector<float> vec_without_0 (0);
        for (int i = 0; i<vec.size(); ++i)
        {
            if(vec[i]>0)
                vec_without_0.push_back(vec[i]);
        }
        std::sort(vec_without_0.begin(), vec_without_0.end());
        if(vec_without_0.size() % 2 == 0)
                return (vec_without_0[vec_without_0.size()/2 - 1] + vec_without_0[vec_without_0.size()/2]) / 2;
        else
                return vec_without_0[vec_without_0.size()/2];
    }
}

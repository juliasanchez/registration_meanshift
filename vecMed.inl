float vecMed(std::vector<float> vec)
{
    if(vec.empty()) return 0;
    else
    {
        std::sort(vec.begin(), vec.end());
        if(vec.size() % 2 == 0)
                return (vec[vec.size()/2 - 1] + vec[vec.size()/2]) / 2;
        else
                return vec[vec.size()/2];
    }
}

void get_axis2(std::vector<std::vector<float>> &clusters1, std::vector<std::vector<float>> &clusters2, std::vector< std::vector <float> >& axis)
{

    float temp=-10000000;
    for (int k2=0; k2<clusters2.size(); k2++)
    {
        double dot1=abs(clusters2[k2][0]*axis[0][0]+clusters2[k2][1]*axis[0][1]+clusters2[k2][2]*axis[0][2]);
        double dot2=abs(clusters2[k2][0]*axis[1][0]+clusters2[k2][1]*axis[1][1]+clusters2[k2][2]*axis[1][2]);

        if (dot1<0.9 && dot2<0.9)
        {
            for (int k1=0; k1<clusters1.size(); k1++)
            {
                double dot=clusters1[k1][0]*clusters2[k2][0]+clusters1[k1][1]*clusters2[k2][1]+clusters1[k1][2]*clusters2[k2][2];
                if(dot>temp)
                {
                    temp=dot;
                    axis[2][0]=clusters2[k2][0];
                    axis[2][1]=clusters2[k2][1];
                    axis[2][2]=clusters2[k2][2];
                }
            }
        }
    }
    if(temp == -10000000) // si le troisième axe n'est pas trouvé on prend une direction perpendiculaire au deux axes
    {
       axis[2][0] = axis[0][1]*axis[1][2]-axis[0][2]*axis[1][1];
       axis[2][1] = axis[0][2]*axis[1][0]-axis[0][0]*axis[1][2];
       axis[2][2] = axis[0][0]*axis[1][1]-axis[0][1]*axis[1][0];
    }

}

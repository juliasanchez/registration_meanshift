void get_axis(std::vector<Eigen::Vector3f> &clusters1, std::vector<Eigen::Vector3f> &clusters2, std::vector<Eigen::Vector3f>& axis)
{
    Eigen::Vector3f axis_perpend = axis[0].cross(axis[1]);
    axis_perpend /= axis_perpend.norm();
    float temp=-10000000;

    for (int k2=0; k2<clusters2.size(); k2++)
    {
        double alpha= acos( abs( clusters2[k2].dot(axis_perpend)/(axis_perpend.norm()*clusters2[k2].norm()) ) ) * 180/ M_PI;

        if (alpha<60)
        {
            for (int k1=0; k1<clusters1.size(); k1++)
            {
                double dot=abs(clusters1[k1].dot(clusters2[k2]));
                if(dot>temp)
                {
                    temp=dot;
                    axis[2]=clusters2[k2];
                }
            }
        }
    }

    if(temp == -10000000)
    {
        std::cout<<"Can not find third angle"<<std::endl<<std::endl;
        axis[2] = axis_perpend;
    }

}

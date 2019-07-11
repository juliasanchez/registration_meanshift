void get_lim_axis(std::set<double> proj_src, std::set<double> proj_tgt, std::vector<float>& axis_lim  )
{
    auto it_end_src = proj_src.end();
    --it_end_src;

    auto it_end_tgt = proj_tgt.end();
    --it_end_tgt;

    axis_lim[0] = std::min(*proj_src.begin(), *proj_tgt.begin());
    axis_lim[1] = std::max(*it_end_src, *it_end_tgt);

    axis_lim[1] += 0.003*(axis_lim[1]-axis_lim[0]);
    axis_lim[0] -= 0.003*(axis_lim[1]-axis_lim[0]);

}

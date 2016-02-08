#include "mav/sensor.h"

namespace asn
{

sensor::sensor(environment_model &env_model):
    environment_model_(env_model)
{

}

sensor::~sensor()
{
}

void sensor::sense(const Vector3f &p)
{
    std::random_device rd;
    std::mt19937 e2(rd());
    auto fp = get_rect(p);

    const int n = 4;
    double dx = fabs(fp[0]-fp[2])/n;
    double dy = fabs(fp[1]-fp[3])/n;
    for(int i=0; i<n; i++)
        for(int j=0; j<n; j++)
        {
            Vector2f v(fp[0]+dx*(0.5+i), fp[1]+dy*(0.5+j));

            auto cel = environment_model_.grid_->find_cell_contains(v);
            if(cel)
            {
                //ROS_INFO("sensing (%d %d)", cel->get_index()[0], cel->get_index()[1]);
                double x[] = {v[0],v[1]};
                std::normal_distribution<> dist(cel->get_ground_truth_value(), active_survey_param::gp_sigma);
                gaussian_field::instance()->add_pattern(x, dist(e2));
            }
        }

    for(auto it=environment_model_.grid_->begin(); it!=environment_model_.grid_->end(); it++)
    {
        //ROS_INFO("updating (%d %d)", (*it)->get_index()[0], (*it)->get_index()[1]);

        double x[] = {(*it)->get_center()[0],(*it)->get_center()[1]};
        (*it)->set_estimated_value(gaussian_field::instance()->f(x));
        (*it)->set_variance(gaussian_field::instance()->var(x));
    }

}

rect sensor::get_rect(const Vector3f &p) const
{
    Vector2f l(p[2]*tan(0.5*active_survey_param::FOV), p[2]*tan(0.5*active_survey_param::FOV));
    return {p[0]-l[0], p[1]-l[1], p[0]+l[0], p[1]+l[1]};
}

}

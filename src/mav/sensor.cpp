#include "mav/sensor.h"
#include <thread>

namespace asn
{

sensor::sensor(environment_model &env_model):
    environment_model_(env_model)
{

}

sensor::~sensor()
{
}

void sensor::sense(const Vector3f &p, std::function<void(std::set<grid_cell::ptr>&, const Vector3f&)> callback)
{
    sensing_locations_.push_back(p);

//    std::thread sensing_thread([this, p, callback]()
//    {
//        //auto t0 = ros::Time::now();
//        this->perform_sense(p, callback);
//        //ROS_INFO_THROTTLE(1,"sensing time: %f", (ros::Time::now()-t0).toSec());
//    });
//    sensing_thread.detach();
    perform_sense(p, callback);
}

grid_cell::ptr sensor::sense_cell(const Vector2f &p)
{
    static std::random_device rd;
    static std::mt19937 e2(rd());

    auto cell = environment_model_.grid_->find_cell_contains(p);
    if(cell)
    {
        auto v = cell->get_center();
        double x[] = {v[0],v[1]};
        std::normal_distribution<> dist(cell->get_ground_truth_value(), active_survey_param::gp_sigma);
        gaussian_field::instance()->add_pattern(x, dist(e2));
    }
    return cell;
}

void sensor::perform_sense(Vector3f p, std::function<void(std::set<grid_cell::ptr>&, const Vector3f&)> callback)
{
    environment_model_.grid_mutex_.lock();

    //auto t0 = ros::Time::now();


    auto fp = get_rect(p);

    Vector2f center(0,0);
    const int n = 2;
    double dx = fabs(fp[0]-fp[2])/n;
    double dy = fabs(fp[1]-fp[3])/n;
    for(int i=0; i<n; i++)
        for(int j=0; j<n; j++)
        {
            Vector2f v(fp[0]+dx*(0.5+i), fp[1]+dy*(0.5+j));
            center += v;
            sense_cell(v);
//            auto cell = environment_model_.grid_->find_cell_contains(v);
//            if(cell)
//            {

//                double x[] = {v[0],v[1]};
//                std::normal_distribution<> dist(cell->get_ground_truth_value(), active_survey_param::gp_sigma);
//                gaussian_field::instance()->add_pattern(x, dist(e2));
//            }
        }

//    center *= 0.25;
//    sense_cell(center);

    rect update_rect = fp;
    //update_rect += rect(-15,-15,15,15);

    std::set<grid_cell::ptr> update_cells;
    environment_model_.grid_->find_cells_in_rect(update_rect, update_cells, false);

    for(auto it=update_cells.begin(); it!=update_cells.end(); it++)
    {
        grid_cell::ptr cell = *it;

        cell->set_covered(true);

//        double x[] = {cell->get_center()[0],cell->get_center()[1]};
//        cell->set_estimated_value(gaussian_field::instance()->f(x));
//        cell->set_variance(gaussian_field::instance()->var(x));
    }


    callback(update_cells, p);

    environment_model_.grid_mutex_.unlock();
}

rect sensor::get_rect(const Vector3f &p) const
{
    Vector2f l(p[2]*tan(0.5*active_survey_param::FOV), p[2]*tan(0.5*active_survey_param::FOV));
    return {p[0]-l[0], p[1]-l[1], p[0]+l[0], p[1]+l[1]};
}

void sensor::draw()
{
    gaussian_field::instance()->draw();
    glColor3f(1,1,0);
    glPointSize(3);
    glBegin(GL_POINTS);
    for(auto &p: sensing_locations_)
        utility::gl_vertex3f(p);
    glEnd();
}

}

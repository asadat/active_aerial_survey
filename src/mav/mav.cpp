
#include "mav/mav.h"

namespace asn
{

mav::mav(environment_model &em, const Vector3f &position):
    environment_model_(em),
    sensor_(em),
    position_(position),
    velocity_(0,0,0),
    goal_(position),
    mav_controller_(),
    stop_(true),
    goal_reached_threshold_(0.5)
{
    behaviour_controller_ = behaviour_controller::ptr(new behaviour_controller(*this));
}

mav::~mav(){}

void mav::sense()
{
    sensor_.sense(position_, [](std::set<grid_cell::ptr>& covered_cells, const Vector3f& p){});
}

void mav::update(const double &dt)
{
    if(!stop_)
        behaviour_controller_->update(dt);
}

void mav::update_state(const double &dt)
{
    velocity_ = mav_controller_.get_velocity(goal_, position_);
    position_ += dt*velocity_;
}

bool mav::at_goal()
{
    return utility::close_enough(goal_, position_, goal_reached_threshold_);
}

void mav::stop()
{
    stop_ = true;
    //set_goal(position_);
}

void mav::resume()
{
    stop_ = false;
}

void mav::draw()
{
    behaviour_controller_->draw();
    sensor_.draw();

    auto fp = sensor_.get_rect(position_);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(1.0);
    glColor3f(0.4, 0.4, 0.4);
    glBegin(GL_QUADS);
    utility::draw_quad(fp);
    glEnd();

    glColor3f(0,0,1);
    glLineWidth(2);
    double l=0.25 * active_survey_param::area_width/32.0;
    glBegin(GL_LINES);
    utility::gl_vertex3f(position_-l*Vector3f(1,0,0));
    utility::gl_vertex3f(position_+l*Vector3f(1,0,0));
    utility::gl_vertex3f(position_-l*Vector3f(0,1,0));
    utility::gl_vertex3f(position_+l*Vector3f(0,1,0));
    glEnd();
}

}

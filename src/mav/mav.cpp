
#include "mav/mav.h"

namespace asn
{

mav::mav(environment_model &em, const Vector3f &position):
    sensor_(em),
    position_(position),
    velocity_(0,0,0),
    goal_(position),
    mav_controller_(),
    goal_reached_threshold_(0.5)
{
    behaviour_controller_ = behaviour_controller::ptr(new behaviour_controller(*this));
}

mav::~mav(){}

void mav::sense()
{
    sensor_.sense(position_);
}

void mav::update(const double &dt)
{
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
    set_goal(position_);
}

void mav::draw()
{
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

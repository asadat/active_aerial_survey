
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
    stop_(false),
    goal_reached_threshold_(0.5)
{
    behaviour_controller_ = behaviour_controller::ptr(new behaviour_controller(*this));
    stop();
}

mav::~mav(){}

void mav::set_goal(const Vector3f &goal)
{
    positions_.push_back(get_position());
    goal_= goal;
}

void mav::sense()
{
    sensor_.sense(position_, [](std::set<grid_cell::ptr>& covered_cells, const Vector3f& p){});
}

bool mav::update(const double &dt)
{
    if(!stop_)
    {
        //ROS_INFO("controller update begin ..");
        return behaviour_controller_->update(dt);
        //ROS_INFO("controller update end ..");
    }

    return true;
}

void mav::update_state(const double &dt)
{
    velocity_ = mav_controller_.get_velocity(goal_, position_);
    Vector3f d = dt*velocity_;

 //   if(d.squaredNorm() > 500)
 //       d.normalize();

    if(utility::distance_squared(position_, goal_) < d.squaredNorm())
        position_ = goal_;
    else
        position_ += d;
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

    if(!stop_)
    {
        auto fp = sensor_.get_rect(position_);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(1.0);
        glColor3f(0.4, 0.4, 0.4);
        glBegin(GL_QUADS);
        utility::draw_quad(fp, 0.2);
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


    glLineWidth(4);
    glColor3f(0.1,1, 0.1);
    glBegin(GL_LINES);
    for(size_t i=0; i+1<positions_.size(); i++)
    {
        if(fabs(active_survey_param::sensing_height-positions_[i][2]) < 0.5 &&
                fabs(active_survey_param::sensing_height-positions_[i+1][2]) < 0.5)
        {
            utility::gl_vertex3f(positions_[i]);
            utility::gl_vertex3f(positions_[i+1]);
        }
    }
    glEnd();

//    glPushAttrib(GL_ENABLE_BIT);
//    glLineStipple(1, 0x5F);
//    glEnable(GL_LINE_STIPPLE);

    glLineWidth(3);
    glColor3f(1, 0.4, 0.4);

    glBegin(GL_LINES);
    for(size_t i=0; i+1<positions_.size(); i++)
    {
        if(fabs(active_survey_param::sensing_height-positions_[i][2]) > 0.5 ||
                fabs(active_survey_param::sensing_height-positions_[i+1][2]) > 0.5)
        {
            if(fabs(active_survey_param::sensing_height-positions_[i][2]) < 0.5)
            {
                glColor3f(0.1,1, 0.1);
            }
            else
            {
                glColor3f(1, 0.4, 0.4);
            }

            utility::gl_vertex3f(positions_[i]);

            if(fabs(active_survey_param::sensing_height-positions_[i+1][2]) < 0.5)
            {
                glColor3f(0.1,1, 0.1);
            }
            else
            {
                glColor3f(1, 0.4, 0.4);

            }
            utility::gl_vertex3f(positions_[i+1]);
        }
    }
    glEnd();
   // glPopAttrib();


    glColor3f(1,1,0);
    glPointSize(10);
    glBegin(GL_POINTS);
    for(size_t i=0; i+1<positions_.size(); i++)
    {
        if(fabs(active_survey_param::sensing_height-positions_[i][2]) > 0.5 ||
                fabs(active_survey_param::sensing_height-positions_[i+1][2]) > 0.5)
        {
            if(fabs(active_survey_param::sensing_height-positions_[i][2]) < 0.5)
            {
                utility::gl_vertex3f(positions_[i], 0.5);
            }

            if(fabs(active_survey_param::sensing_height-positions_[i+1][2]) < 0.5)
            {
                utility::gl_vertex3f(positions_[i+1], 0.5);
            }
        }
    }
    glEnd();



}

}

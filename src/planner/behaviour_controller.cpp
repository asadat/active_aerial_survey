#include "planner/behaviour_controller.h"
#include "mav/mav.h"

namespace asn
{

behaviour_controller::behaviour_controller(mav &m):
    mav_(m),
    behaviour_planner_(new behaviour_planner(m)),
    sensing_(false)
{
     //= behaviour_planner::ptr(new behaviour_planner(mav_));
    last_sensing_pos_ = mav_.get_position();
}

behaviour_controller::~behaviour_controller()
{
}

void behaviour_controller::update(const double &dt)
{
    if(!mav_.at_goal())
    {
        mav_.update_state(dt);
        if(sensing_)
        {
            if((last_sensing_pos_-mav_.get_position()).squaredNorm() > active_survey_param::min_footprint*5*active_survey_param::min_footprint*5)
            {
                last_sensing_pos_ = mav_.get_position();
                mav_.sensor_.sense(mav_.get_position(), [this](std::set<grid_cell::ptr>& covered_cells){this->behaviour_planner_->sensing_callback(covered_cells);});
            }
        }
    }
    else
    {
      if(waypoint_)
      {
          waypoint_->on_reached_waypoint(waypoint_);

          auto wp_action = waypoint_->get_action();
          switch(wp_action)
          {
            case waypoint::action::START_SENSING:
              sensing_ = true;
              break;
            case waypoint::action::STOP_SENSING:
              sensing_ = false;
              break;
            case waypoint::action::NONE:
            default:
              break;
          }
      }

      waypoint_ = behaviour_planner_->get_next_waypoint();
      if(waypoint_)
      {
          mav_.set_goal(waypoint_->get_position());
      }
      else
      {
          ROS_INFO_THROTTLE(2,"behaviour controller: invalid waypoint pointer!");
      }
    }
}

void behaviour_controller::draw()
{
    behaviour_planner_->draw();
}

}

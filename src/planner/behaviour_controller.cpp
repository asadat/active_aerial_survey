#include "planner/behaviour_controller.h"
#include "mav/mav.h"

namespace asn
{

behaviour_controller::behaviour_controller(mav &m):
    mav_(m),
    behaviour_planner_(new behaviour_planner(m)),
    sensing_(false),
    avaiable_flight_time_(active_survey_param::time_limit)
{
    behaviour_planner_->set_get_available_time([this](){return this->get_available_flight_time();});
    last_sensing_pos_ = mav_.get_position();
    last_pos_flight_time_update = last_sensing_pos_;
}

behaviour_controller::~behaviour_controller()
{
}

void behaviour_controller::start_sensing(bool override_min_travel_dist)
{
    if(sensing_)
    {

        if(override_min_travel_dist || utility::distance_squared(last_sensing_pos_,mav_.get_position()) >
                active_survey_param::min_footprint*5*active_survey_param::min_footprint*5)
        {
            update_available_flight_time(false);
            last_sensing_pos_ = mav_.get_position();
            mav_.sensor_.sense(mav_.get_position(),
                               [this](std::set<grid_cell::ptr>& covered_cells, const Vector3f& p)
            {
                this->behaviour_planner_->sensing_callback(covered_cells, p);
            });
        }
    }
    else
    {
        update_available_flight_time(false);
    }
}

double behaviour_controller::get_available_flight_time()
{
    double aft;
    flight_time_mutex_.lock();
    aft = avaiable_flight_time_;
    flight_time_mutex_.unlock();
    return aft;
}

void behaviour_controller::reduce_available_flight_time(double dt)
{
    flight_time_mutex_.lock();
    avaiable_flight_time_ -= dt;
    flight_time_mutex_.unlock();
}

void behaviour_controller::update_available_flight_time(bool turning_point)
{
    if(turning_point)
        reduce_available_flight_time(active_survey_param::turning_time);

    auto mav_pos = mav_.get_position();
    double travelled_dist = utility::distance(mav_pos, last_pos_flight_time_update);

    reduce_available_flight_time(travelled_dist/active_survey_param::speed);

    last_pos_flight_time_update = mav_pos;

    //ROS_INFO_THROTTLE(2,"flight time: %f", avaiable_flight_time_);
}

void behaviour_controller::update(const double &dt)
{
    waypoint::action requested_action =  behaviour_planner_->get_controller_action();
    behaviour_planner_->reset_controller_action();

    bool switch_to_next_waypoint = false;

    if(requested_action != waypoint::action::INTERRUPT_WAYPOINT)
    {
        if(!mav_.at_goal())
        {
            mav_.update_state(dt);
            start_sensing(false);
        }
        else
        {
          switch_to_next_waypoint = true;

          if(waypoint_)
          {
              auto wp_action = waypoint_->get_action();
              switch(wp_action)
              {
                case waypoint::action::START_SENSING:
                  sensing_ = true;
                  break;
                case waypoint::action::STOP_SENSING:
                  //start_sensing(true);
                  sensing_ = false;
                  break;
                case waypoint::action::NONE:
                default:
                  break;
              }
              waypoint_->on_reached_waypoint(waypoint_);
          }
      }
    }
    else
    {
        switch_to_next_waypoint = true;
    }


    if(switch_to_next_waypoint)
    {
        if(requested_action == waypoint::action::STOP_SENSING)
            last_sensing_pos_ = mav_.get_position();

        update_available_flight_time(true);

        waypoint_ = behaviour_planner_->get_next_waypoint();
        if(waypoint_)
        {
            auto wp_action = waypoint_->get_on_set_action();
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

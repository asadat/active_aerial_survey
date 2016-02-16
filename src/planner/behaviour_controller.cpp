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

    reduce_available_flight_time(travelled_dist/active_survey_param::average_speed);

    last_pos_flight_time_update = mav_pos;

    //ROS_INFO_THROTTLE(2,"flight time: %f", avaiable_flight_time_);
}

void behaviour_controller::update_sensed_cells(waypoint::ptr prev_wp, waypoint::ptr next_wp)
{
    if(prev_wp->get_type() != waypoint::type::HIGH_RESOLUTION ||
            next_wp->get_type() != waypoint::type::HIGH_RESOLUTION)
        return;

    Vector3f p = prev_wp->get_position();
    Vector3f ep = next_wp->get_position();
    Vector3f dir = (ep-p).normalized();

    auto fp = mav_.sensor_.get_rect(p);
    double step = std::min(fabs(fp[0]-fp[2]), fabs(fp[1]-fp[3]));
    bool flag = true;

    do
    {
        if(utility::distance_squared(p,ep) > step*step)
        {
            p += step*dir;
        }
        else
        {
            p = ep;
            flag = false;
        }

        fp = mav_.sensor_.get_rect(p);
        std::set<grid_cell::ptr> cells;
        mav_.get_grid().find_cells_in_rect(fp, cells, false);
        for(auto c:cells)
            c->set_sensed(true);

    }
    while(flag);
}

void behaviour_controller::calculate_performace()
{
    size_t sensed_poly_cells = 0;
    size_t sensed_targets = 0;

    for(auto it=mav_.get_grid().begin(); it !=mav_.get_grid().end(); ++it)
    {
        if((*it)->is_covered() && (*it)->is_sensed() && (*it)->is_target())
            sensed_targets++;

        if((*it)->is_covered() && (*it)->is_sensed() && (*it)->has_label())
            sensed_poly_cells++;
    }

    auto cell_size = mav_.get_grid().get_cell_size();
    double sensed_target_area = sensed_targets * cell_size[0] * cell_size[1];
    double sensed_poly_area = sensed_poly_cells * cell_size[0] * cell_size[1];

    ROS_INFO("RESULT -> sensed polygons: %.1f sensed target: %.1f", sensed_poly_area, sensed_target_area);
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

        waypoint::ptr prev_waypoint = waypoint_;
        waypoint_ = behaviour_planner_->get_next_waypoint();

        if(waypoint_ && prev_waypoint)
            update_sensed_cells(prev_waypoint, waypoint_);

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
            calculate_performace();
            mav_.stop();
            ROS_INFO_THROTTLE(2,"behaviour controller: invalid waypoint pointer! (remaining flight time: %.fs)", get_available_flight_time());
        }
    }

}

void behaviour_controller::draw()
{
    behaviour_planner_->draw();
}

}

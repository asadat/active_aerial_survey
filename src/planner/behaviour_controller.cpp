#include "planner/behaviour_controller.h"
#include "mav/mav.h"

namespace asn
{

behaviour_controller::behaviour_controller(mav &m):
    mav_(m),
    behaviour_planner_(std::make_shared<behaviour_planner>(m)),
    sensing_(false),
    waypoint_(nullptr),
    previous_waypoint_(nullptr),
    avaiable_flight_time_(active_survey_param::time_limit)
{
    behaviour_planner_->set_get_available_time([this](){return this->get_available_flight_time();});
    last_sensing_pos_ = mav_.get_position();
    last_pos_flight_time_update = last_sensing_pos_;
}

behaviour_controller::~behaviour_controller()
{
}

void behaviour_controller::start_sensing(bool override_min_travel_dist, const waypoint::ptr &reached_waypoint)
{
    if(sensing_)
    {

        if(override_min_travel_dist || utility::distance_squared(last_sensing_pos_,mav_.get_position()) >
                active_survey_param::min_footprint*5*active_survey_param::min_footprint*5)
        {
            //ROS_INFO("calling sensing ...");
            //update_available_flight_time(override_min_travel_dist);
            last_sensing_pos_ = mav_.get_position();
            mav_.sensor_.sense(mav_.get_position(),
                               [this, override_min_travel_dist, &reached_waypoint](std::set<grid_cell::ptr>& covered_cells, const Vector3f& p)
            {
                //ROS_INFO("callback sensing ...");
                this->behaviour_planner_->sensing_callback(covered_cells, p,
                                                           /*override_min_travel_dist && behaviour_planner_->get_waypoint_count()==1*/
                                                           reached_waypoint);
            });

            //ROS_INFO("returning from sensing ...");
        }
    }
    else
    {
        //update_available_flight_time(override_min_travel_dist);
    }
}

double behaviour_controller::get_available_flight_time()
{
    double aft;
    //flight_time_mutex_.lock();
    aft = avaiable_flight_time_;
    //flight_time_mutex_.unlock();
    return aft;
}

void behaviour_controller::reduce_available_flight_time(double dt)
{
    //flight_time_mutex_.lock();
    avaiable_flight_time_ -= dt;
    //flight_time_mutex_.unlock();
}

void behaviour_controller::update_available_flight_time(bool turning_point)
{
    if(turning_point)
        reduce_available_flight_time(active_survey_param::turning_time);

    auto mav_pos = mav_.get_position();
    double travelled_dist = utility::distance(mav_pos, last_pos_flight_time_update);

    double dh = fabs(mav_pos[2]-last_pos_flight_time_update[2]);
    if(dh> 0.1)
    {
        reduce_available_flight_time(2.0*travelled_dist/active_survey_param::average_speed);
    }
    else
    {
        reduce_available_flight_time(travelled_dist/active_survey_param::average_speed);
    }

    last_pos_flight_time_update = mav_pos;

    //ROS_INFO_THROTTLE(2,"flight time: %f", avaiable_flight_time_);
}

void behaviour_controller::update_sensed_cells(waypoint::ptr prev_wp, waypoint::ptr next_wp)
{
    static size_t cumulative_value=0;

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
        {
            if(!c->is_sensed() && c->has_label())
                cumulative_value++;

            c->set_sensed(true);

            double sensed_time = active_survey_param::time_limit - get_available_flight_time();
            if(sensed_time >=0)
                c->set_sensed_time(sensed_time);
            else
                ROS_ERROR("**** sensed time < 0");
        }

    }
    while(flag);

  //  ROS_INFO("CUMULATIVE VALUE: %.1f %ld ", active_survey_param::time_limit - get_available_flight_time() , cumulative_value);
}

void behaviour_controller::calculate_performace()
{
    auto cell_size = mav_.get_grid().get_cell_size();

    size_t sensed_poly_cells = 0;
    size_t sensed_targets = 0;
    double cell_area = cell_size[0] * cell_size[1];
    double discounted_reward = 0;
    double true_discounted_reward = 0;

    for(auto it=mav_.get_grid().begin(); it !=mav_.get_grid().end(); ++it)
    {
        if((*it)->is_covered() && (*it)->is_sensed() && (*it)->is_target())
            sensed_targets++;

        if((*it)->is_covered() && (*it)->is_sensed() && (*it)->has_label())
        {
            sensed_poly_cells++;
            discounted_reward += cell_area * pow(active_survey_param::discount_factor, (*it)->get_sensed_time());
        }

        if((*it)->is_covered() && (*it)->is_sensed() && (*it)->is_true_target(active_survey_param::non_ros::target_threshold))
        {
            true_discounted_reward += cell_area * pow(active_survey_param::discount_factor, (*it)->get_sensed_time());
        }
    }

    //double sensed_target_area = sensed_targets * cell_area;
    double sensed_poly_area = sensed_poly_cells * cell_area;

    ROS_INFO("RESULT %s -> discounted_reward: %.3f sensed polygons: %.1f true_d_r: %.1f Interesting: %.1f Patches: %d exploit_rate: %.3f seed: %.1f",
             active_survey_param::policy.c_str(), discounted_reward, sensed_poly_area, true_discounted_reward,
             active_survey_param::percent_interesting, active_survey_param::patches,
             active_survey_param::exploitation_rate, active_survey_param::random_seed);
}

bool behaviour_controller::update(const double &dt)
{
    //ROS_INFO("controller 1");
    waypoint::action requested_action =  behaviour_planner_->get_controller_action();
    behaviour_planner_->reset_controller_action();

    bool switch_to_next_waypoint = false;

    if(requested_action != waypoint::action::INTERRUPT_WAYPOINT)
    {
        update_available_flight_time(false);

        if(!mav_.at_goal())
        {
            mav_.update_state(dt);
            start_sensing(false, nullptr);
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
                    start_sensing(true, waypoint_);
                    break;
                case waypoint::action::STOP_SENSING:
                    start_sensing(true, waypoint_);
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

    //ROS_INFO("controller 2");

    if(switch_to_next_waypoint)
    {
        if(requested_action == waypoint::action::STOP_SENSING)
            last_sensing_pos_ = mav_.get_position();

        update_available_flight_time(true);

        if(waypoint_ && previous_waypoint_)
            update_sensed_cells(previous_waypoint_, waypoint_);

        previous_waypoint_ = waypoint_;
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
            calculate_performace();
            ROS_INFO_THROTTLE(2,"behaviour controller: invalid waypoint pointer! (%.fs)", get_available_flight_time());
            mav_.stop();

            return false;
        }
    }

    //ROS_INFO("controller 3");

    return true;
}

void behaviour_controller::draw()
{
    behaviour_planner_->draw();
}

}

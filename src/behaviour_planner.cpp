#include "behaviour_planner.h"
#include "mav.h"

namespace asn
{

behaviour_planner::behaviour_planner(mav &m):
    mav_(m)
{
    //generate_test_waypoint();
}

behaviour_planner::~behaviour_planner()
{
}

void behaviour_planner::generate_test_waypoints()
{
    for(int i=0; i<10; i++)
    {
        waypoint::ptr wp(new waypoint({(float)i,5.0f*(i%2), 10.0}));
        wp->set_waypoint_call_back([](waypoint::ptr wp_ptr)
        {
            ROS_INFO("Waypoint callback: reached test waypoint %f %f %f",
                    wp_ptr->get_position()[0],
                    wp_ptr->get_position()[1],
                    wp_ptr->get_position()[2]);
        });

        plan_.push_back(wp);
    }
}

waypoint::ptr behaviour_planner::get_next_waypoint()
{
    if(plan_.empty())
        return nullptr;
    else
        return plan_.pop_next_waypoint();
}

}

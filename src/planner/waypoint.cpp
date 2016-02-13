#include "planner/waypoint.h"

namespace asn
{
waypoint::waypoint(const Vector3f &pos):
    pos_(pos),
    action_(action::NONE),
    on_set_action_(action::NONE),
    type_(type::COARSE)
{

}

void waypoint::on_reached_waypoint(ptr waypoint_ptr)
{
    if(waypoint_call_back_)
        waypoint_call_back_(waypoint_ptr);
}

}

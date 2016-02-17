#include "planner/waypoint.h"

namespace asn
{
waypoint::waypoint(const Vector3f &pos):
    pos_(pos),
    action_(action::NONE),
    on_set_action_(action::NONE),
    type_(type::COARSE),
    flags_(0)
{

}

void waypoint::on_reached_waypoint(ptr waypoint_ptr)
{
    if(waypoint_call_back_)
        waypoint_call_back_(waypoint_ptr);
}

bool waypoint::get_flag(const flag &f) const
{
    return flags_ & static_cast<char>(f);
}

void waypoint::set_flag(const flag &f)
{
    flags_ |= static_cast<char>(f);
}

void waypoint::unset_flag(const flag &f)
{
    flags_&= ~(1<<static_cast<char>(f));
}

}

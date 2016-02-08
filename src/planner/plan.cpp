#include "planner/plan.h"
#include "stdexcept"

namespace asn
{

plan::plan()
{

}

plan::~plan()
{

}

waypoint::ptr plan::pop_next_waypoint()
{
    if(waypoints_.empty())
        throw std::range_error("Tried to pop waypoint an empty plan");

    auto wp = waypoints_.back();
    waypoints_.pop_back();
    return wp;
}

}

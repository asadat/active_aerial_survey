#pragma once

#include <memory>
#include <vector>
#include "planner/waypoint.h"

namespace asn
{

class plan
{
public:
    typedef std::shared_ptr<plan> ptr;
    typedef std::vector<waypoint::ptr>::iterator waypoint_iterator;
    typedef std::vector<waypoint::ptr>::const_iterator waypoint_const_iterator;

public:
    plan();
    virtual ~plan();

    waypoint::ptr front() const {return waypoints_.front();}
    waypoint::ptr back() const {return waypoints_.back();}

    waypoint_const_iterator begin() const {return waypoints_.begin();}
    waypoint_const_iterator end() const {return waypoints_.end();}
    bool empty() const {return waypoints_.empty();}

    waypoint::ptr pop_next_waypoint();
    void push_front(waypoint::ptr waypoint){waypoints_.push_back(waypoint);}
    void push_back(waypoint::ptr waypoint){waypoints_.insert(waypoints_.begin(),waypoint);}

private:
    std::vector<waypoint::ptr> waypoints_;
};

}

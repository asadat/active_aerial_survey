#pragma once

#include <memory>
#include "plan.h"

namespace asn
{
class mav;

class behaviour_planner
{
public:
    typedef std::shared_ptr<behaviour_planner> ptr;

public:
    behaviour_planner(mav &m);
    virtual ~behaviour_planner();

    waypoint::ptr get_next_waypoint();

private:
    void generate_test_waypoints();

    mav &mav_;
    plan plan_;
};

}

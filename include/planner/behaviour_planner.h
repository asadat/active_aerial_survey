#pragma once

#include <memory>
#include "planner/plan.h"

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

    void draw();
    void sensing_callback();

private:
    void greedy();
    void semi_greedy();
    void two_stage();

    void generate_coarse_survey();
    void generate_test_waypoints();

    mav &mav_;
    plan plan_;

    waypoint::ptr last_waypoint;
};

}

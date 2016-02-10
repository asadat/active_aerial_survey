#pragma once

#include <memory>
#include <map>
#include <mutex>

#include "math/graph.h"
#include "environment_model/grid_cell.h"
#include "environment_model/grid_segment.h"
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
    void sensing_callback(std::set<grid_cell::ptr>& covered_cells);

    void set_get_available_time(std::function<double(void)> f){get_available_time = f;}

private:
    void greedy();
    void semi_greedy();
    void two_stage();

    void generate_coarse_survey();
    void generate_test_waypoints();

    mav &mav_;
    plan plan_;

    std::map<grid_cell_base::label, grid_segment::ptr> segments_;
    waypoint::ptr last_waypoint;

    std::set<grid_cell::ptr> covered_cells_;
    std::mutex components_mutex_;
    std::vector<graph::ptr> components_;
    graph::ptr  graph_;

    std::function<double(void)> get_available_time;
};

}

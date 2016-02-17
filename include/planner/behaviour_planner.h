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
    void sensing_callback(std::set<grid_cell::ptr>& covered_cells, const Vector3f &sensing_position, bool last_coarse_survey_waypoint);

    void set_get_available_time(std::function<double(void)> f){get_available_time_ = f;}

    waypoint::action get_controller_action() const {return controller_action_;}
    void reset_controller_action() {controller_action_ = waypoint::action::NONE;}

    inline size_t get_waypoint_count() const {return plan_.size();}
private:
    void greedy();
    void semi_greedy(bool reached_last_coarse_survey_waypoint);
    void two_stage();

    void generate_coarse_survey();
    void generate_test_waypoints();

    template<typename cell_iterator>
    void update_segments(const cell_iterator &begin_it, const cell_iterator &end_it);

    template<typename cell_iterator>
    void update_grid_gp(const cell_iterator &begin_it, const cell_iterator &end_it);

    void plan_sensing_tour(std::vector<grid_segment::ptr> &segments, const Vector3f &pos,
                           const double &available_flight_time, const waypoint::ptr &cur_waypoint ,plan & cur_plan);

    bool construct_coverage_plan(grid_segment::ptr segment, const Vector3f &pos,
                                 const double &available_flight_time, const waypoint::ptr &cur_waypoint ,
                                 plan & cur_plan, plan& coverage_plan);
    mav &mav_;
    plan plan_;

    std::map<grid_cell_base::label, grid_segment::ptr> segments_;
    waypoint::ptr last_waypoint_;
    Vector3f last_sensing_position_;

    waypoint::action controller_action_;

    std::vector<grid_cell::ptr> uncertain_cells_;
    std::set<grid_cell::ptr> covered_cells_;
    std::mutex components_mutex_;
    std::vector<graph::ptr> components_;
    //std::vector<graph::ptr> planned_components_;

    graph::ptr  graph_;

    std::function<double(void)> get_available_time_;

    std::set<grid_cell> sensed_grid_cells_;
};

}

#pragma once
#include <memory>
#include "planner/behaviour_planner.h"

namespace asn
{

class mav;

class behaviour_controller
{
public:
    typedef std::shared_ptr<behaviour_controller> ptr;

public:
    behaviour_controller(mav &m);
    virtual ~behaviour_controller();

    void update(const double &dt);
    void draw();

    double get_available_flight_time();

private:
    mav & mav_;
    behaviour_planner::ptr behaviour_planner_;

    void start_sensing(bool override_min_travel_dist, const waypoint::ptr &reached_waypoint);
    void reduce_available_flight_time(double dt);
    void update_available_flight_time(bool turning_point);
    void update_sensed_cells(waypoint::ptr prev_wp, waypoint::ptr next_wp);
    void calculate_performace();

    Vector3f last_pos_flight_time_update;

    waypoint::ptr waypoint_;
    Vector3f last_sensing_pos_;
    bool sensing_;

    std::mutex flight_time_mutex_;
    double avaiable_flight_time_;

};
}

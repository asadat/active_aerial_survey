#include "planner/plan.h"
#include "stdexcept"
#include "utility.h"
#include "active_survey_param.h"

namespace asn
{

plan::plan()
{

}

plan::~plan()
{

}

waypoint::ptr plan::pop_last_waypoint()
{
    if(waypoints_.empty())
        throw std::range_error("Tried to pop last waypoint in an empty plan");

    auto wp = waypoints_.back();
    waypoints_.pop_back();
    return wp;
}

waypoint::ptr plan::pop_next_waypoint()
{
    if(waypoints_.empty())
        throw std::range_error("Tried to pop waypoint an empty plan");

    auto wp = waypoints_.front();
    waypoints_.erase(waypoints_.begin());
    return wp;
}

double plan::get_overall_cost(waypoint::ptr from)
{
    if(empty())
        return 0.0;

    double cost=0;

    if(from)
    {
        cost += utility::distance(from->get_position(), front()->get_position())/active_survey_param::average_speed;
        cost += active_survey_param::turning_time;
    }

    auto prev = begin();
    for(auto it=begin()+1; it != end(); ++it)
    {
        cost += utility::distance((*prev)->get_position(), (*it)->get_position())/active_survey_param::average_speed;
        cost += active_survey_param::turning_time;
    }

    return cost;
}

double plan::cost(const Vector3f &cur_pos, const plan &next_plan)
{
    double dist_cost=0;
    Vector3f v =cur_pos;


    for(auto it=begin(); it!=end(); ++it)
    {
        auto p=(*it)->get_position();
        if(fabs(v[2]-p[2] > 0.1))
            dist_cost += 2.0*utility::distance(v, p)/active_survey_param::average_speed;
        else
            dist_cost += utility::distance(v, p)/active_survey_param::average_speed;

        v = (*it)->get_position();
    }

    for(auto it=next_plan.begin(); it!=next_plan.end(); ++it)
    {
        auto p=(*it)->get_position();
        if(fabs(v[2]-p[2] > 0.1))
            dist_cost += 2.0*utility::distance(v, p)/active_survey_param::average_speed;
        else
            dist_cost += utility::distance(v, p)/active_survey_param::average_speed;

        v = (*it)->get_position();
    }

    return dist_cost + (1+waypoints_.size()+next_plan.waypoints_.size())
            *active_survey_param::turning_time;
}

}

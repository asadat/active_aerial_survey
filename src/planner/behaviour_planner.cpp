#include "planner/behaviour_planner.h"
#include "mav/mav.h"
#include "planner/trajectory_planner.h"

namespace asn
{

behaviour_planner::behaviour_planner(mav &m):
    mav_(m),
    last_waypoint(nullptr)
{
    //generate_test_waypoint();
    generate_coarse_survey();
}

behaviour_planner::~behaviour_planner()
{
}

void behaviour_planner::generate_test_waypoints()
{
    for(int i=0; i<10; i++)
    {
        waypoint::ptr wp(new waypoint({(float)i,5.0f*(i%2), 10.0}));
        wp->set_waypoint_call_back([](waypoint::ptr wp_ptr)
        {
            ROS_INFO("Waypoint callback: reached test waypoint %f %f %f",
                    wp_ptr->get_position()[0],
                    wp_ptr->get_position()[1],
                    wp_ptr->get_position()[2]);
        });

        plan_.push_back(wp);
    }
}

void behaviour_planner::generate_coarse_survey()
{
    polygon area_poly;
    mav_.environment_model_.get_environment_polygon(area_poly);

    trajectory_planner::trajectory t;
    trajectory_planner::plan_coverage_rectangle(area_poly, t, {20.0, 20.0}, 10.0);

    for(auto &tp: t)
    {
        waypoint::ptr wp(new waypoint(tp));
        wp->set_waypoint_call_back([](waypoint::ptr wp_ptr)
        {
            ROS_INFO("Waypoint callback: reached test waypoint %f %f %f",
                    wp_ptr->get_position()[0],
                    wp_ptr->get_position()[1],
                    wp_ptr->get_position()[2]);
        });
        plan_.push_back(wp);
    }
}

waypoint::ptr behaviour_planner::get_next_waypoint()
{
    if(plan_.empty())
        return nullptr;
    else
    {
        last_waypoint = plan_.pop_next_waypoint();
        return last_waypoint;
    }
}

void behaviour_planner::sensing_callback(std::set<grid_cell::ptr>& covered_cells)
{
    for(auto it=covered_cells.begin(); it!=covered_cells.end(); it++)
    {
        grid_cell::ptr cell = *it;
        if(cell->is_target() && !cell->has_label())
        {
            grid_segment::ptr gs(new grid_segment(mav_.get_grid(), cell, grid_cell_base::generate_label()));
            gs->grow([this](const grid_cell_base::label &l) -> grid_segment::ptr
            {
                auto it = this->segments_.find(l);
                if(it != segments_.end())
                    return it->second;
                else
                    return nullptr;
            });

            segments_.insert(std::pair<grid_cell_base::label, grid_segment::ptr>(cell->get_label(), gs));
        }
    }

    if(active_survey_param::policy == "greedy")
        greedy();
    else if(active_survey_param::policy == "delayed_greedy")
        semi_greedy();
    else
        two_stage();
}

void behaviour_planner::draw()
{
    if(!plan_.empty())
    {
        glLineWidth(3);
        glColor3f(0.1,0.7, 0.3);
        glBegin(GL_LINES);

        auto it=plan_.begin();

        utility::gl_vertex3f((*it)->get_position());

        for(it++; it!=plan_.end(); it++)
        {
            utility::gl_vertex3f((*it)->get_position());
            utility::gl_vertex3f((*it)->get_position());
        }

        if(last_waypoint)
        {
            utility::gl_vertex3f(last_waypoint->get_position());
        }

        glEnd();
    }


    for(auto it= segments_.begin(); it!=segments_.end(); it++)
    {
        grid_segment::ptr sg = it->second;
        auto some_cell_center = (*sg->begin())->get_center();

        glColor3f(0.2, 0.2, 1);
        glPointSize(9);
        glBegin(GL_POINTS);
        utility::gl_vertex3f(some_cell_center, 0.2);
        glEnd();

        glColor3f(0,.3,1);
        glLineWidth(3);
        glBegin(GL_LINES);
        for(auto i=sg->begin_neighbour(); i!=sg->end_neighbour(); i++)
        {
            grid_segment::ptr sg_n = segments_[*i];

            utility::gl_vertex3f(some_cell_center, 0.2);
            utility::gl_vertex3f((*sg_n->begin())->get_center(), 0.2);
        }
        glEnd();
    }
}

void behaviour_planner::greedy()
{

}

void behaviour_planner::semi_greedy()
{

}

void behaviour_planner::two_stage()
{

}

}

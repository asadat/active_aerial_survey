#include "planner/behaviour_planner.h"
#include "mav/mav.h"
#include "planner/trajectory_planner.h"

namespace asn
{

behaviour_planner::behaviour_planner(mav &m):
    mav_(m),
    last_waypoint(nullptr)
{
    graph_ = std::make_shared<graph>();
    generate_coarse_survey();
}

behaviour_planner::~behaviour_planner()
{
}

void behaviour_planner::generate_test_waypoints()
{
    for(int i=0; i<10; i++)
    {
        waypoint::ptr wp = std::make_shared<waypoint>(Vector3f({(float)i,5.0f*(i%2), 10.0}));
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
    trajectory_planner::plan_coverage_rectangle(area_poly, t, {40.0, 40.0}, 20.0);

    for(auto &tp: t)
    {
        waypoint::ptr wp = std::make_shared<waypoint>(tp);
        wp->set_waypoint_call_back([](waypoint::ptr wp_ptr)
        {
            ROS_INFO("Waypoint callback: reached test waypoint %f %f %f",
                    wp_ptr->get_position()[0],
                    wp_ptr->get_position()[1],
                    wp_ptr->get_position()[2]);
        });

        wp->set_action(waypoint::action::START_SENSING);

        plan_.push_back(wp);
    }

    //stop sensing after the last waypoint
    plan_.back()->set_action(waypoint::action::STOP_SENSING);

    //insert home waypoint
    auto home_wp = std::make_shared<waypoint>(mav_.get_position());
    home_wp->set_action(waypoint::action::NONE);
    plan_.push_back(home_wp);

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
    for(auto cell:covered_cells)
        covered_cells_.insert(cell);

    ROS_INFO("#unprocessed cells: %ld", covered_cells_.size());

    if(covered_cells_.size() < 1000)
        return;

    for(auto cell:covered_cells_)
    {
        double x[] = {cell->get_center()[0],cell->get_center()[1]};
        cell->set_estimated_value(gaussian_field::instance()->f(x));
        cell->set_variance(gaussian_field::instance()->var(x));
    }

    for(auto cell:covered_cells_)
    {        
        if(cell->is_target() && !cell->has_label())
        {
            grid_segment::ptr gs = std::make_shared<grid_segment>(mav_.get_grid(), cell, grid_cell_base::generate_label());

            segments_.insert(std::pair<grid_cell_base::label, grid_segment::ptr>(cell->get_label(), gs));

            gs->grow([this](const grid_cell_base::label &l) -> grid_segment::ptr
            {
                auto it = this->segments_.find(l);
                if(it != segments_.end())
                    return it->second;
                else
                {
                    ROS_WARN("unregistered label requested !!!!");
                    return nullptr;
                }
            });

            gs->find_approximate_polygon();
            gs->find_convexhull();

            graph_->add_node(std::static_pointer_cast<graph_node>(gs));
        }
    }

    components_mutex_.lock();

    components_.clear();
    graph::get_components(graph_, components_);

    components_mutex_.unlock();

    covered_cells_.clear();

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

        if(last_waypoint)
        {
            utility::gl_vertex3f(last_waypoint->get_position());
        }
        else
        {
            utility::gl_vertex3f(mav_.get_position());
        }

        for(; it+1!=plan_.end(); it++)
        {
            utility::gl_vertex3f((*it)->get_position());
            utility::gl_vertex3f((*it)->get_position());
        }


        glEnd();
    }


    components_mutex_.lock();
//    for(auto &c: components_)
//    {
//        c->draw();
//    }

    for(auto &gsp:segments_)
    {
        grid_segment::ptr gs = gsp.second;
        glLineWidth(5);
        glColor3f(0.6,0.5,0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glBegin(GL_POLYGON);
        //for(auto it=gs->begin_approx_poly(); it!= gs->end_approx_poly(); it++)
        for(auto it=gs->begin_convexhull(); it!= gs->end_convexhull(); it++)
            utility::gl_vertex3f(*it, 0.3);
        glEnd();
    }
    components_mutex_.unlock();
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

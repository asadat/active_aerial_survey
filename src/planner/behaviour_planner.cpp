#include "planner/behaviour_planner.h"
#include "mav/mav.h"
#include "planner/trajectory_planner.h"

namespace asn
{

behaviour_planner::behaviour_planner(mav &m):
    mav_(m),
    last_waypoint_(nullptr),
    controller_action_(waypoint::action::NONE)
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
            ROS_INFO("WP CB: %f %f %f",
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
        last_waypoint_ = plan_.pop_next_waypoint();
        return last_waypoint_;
    }
}

template<typename cell_iterator>
void behaviour_planner::update_grid_gp(const cell_iterator &begin_it, const cell_iterator &end_it)
{
    for(auto it =begin_it; it!=end_it; ++it)
    {
        auto &cell = *it;
        double x[] = {cell->get_center()[0],cell->get_center()[1]};
        cell->set_estimated_value(gaussian_field::instance()->f(x));
        cell->set_variance(gaussian_field::instance()->var(x));
    }
}


void behaviour_planner::sensing_callback(std::set<grid_cell::ptr>& covered_cells, const Vector3f &sensing_position)
{
    last_sensing_position_ = sensing_position;

    for(auto cell:covered_cells)
        covered_cells_.insert(cell);

   ROS_INFO("#unprocessed cells: %ld", covered_cells_.size());

    if(covered_cells_.size() > mav_.get_grid().cells_count()*active_survey_param::exploitation_rate)
    {
        if(active_survey_param::speed < 100)
            update_grid_gp(mav_.get_grid().begin(), mav_.get_grid().end());


        //update_grid_gp();
        //update_segments();

        if(active_survey_param::policy == "greedy")
            greedy();
        else if(active_survey_param::policy == "delayed_greedy")
            semi_greedy();
        else
            two_stage();
    }
}


template<typename cell_iterator>
void behaviour_planner::update_segments(const cell_iterator &begin_it, const cell_iterator &end_it)
{
    for(auto cell_it=begin_it; cell_it!=end_it; ++cell_it)
    {
        auto cell = *cell_it;
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
                    ROS_WARN("unregistered label requested %u !!!!", l);
                    for(auto pr:segments_)
                        ROS_WARN("SEGMENT: %u", pr.second->get_label());

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

    if(!components_.empty())
    {
        for(auto cit=components_.begin(); cit!=components_.end(); ++cit)
        {
            graph::ptr component = *cit;
            //ROS_INFO("merging component size: %ld", component->size());
            auto s_it=component->begin();
            grid_segment::ptr first_segment = std::static_pointer_cast<grid_segment>(*s_it);

            while(component->size()>1)
            {
                auto seg_it = component->begin();
                ++seg_it;
                grid_segment::ptr mergin_seg = std::static_pointer_cast<grid_segment>(*seg_it);
                segments_.erase(mergin_seg->get_label());

                //ROS_WARN("mergin %u with %u", first_segment->get_label(), mergin_seg->get_label());
                grid_segment::merge_with_segment(first_segment, mergin_seg, graph_);
                component->remove_left_alone_node(*seg_it);
            }

            //ROS_INFO("finished merging component size: %ld", component->size());
        }
    }

//    ROS_INFO("#components: %ld #segments: %ld", components_.size(), segments_.size());
//    for(auto s:segments_)
//        ROS_ERROR("#cells %ld in segment %u", s.second->get_approx_poly_vertices_count(), s.second->get_label());

    components_mutex_.unlock();

}

void behaviour_planner::draw()
{
    if(!plan_.empty())
    {
        glLineWidth(3);
        glColor3f(0.1,0.7, 0.3);
        glBegin(GL_LINES);

        auto it=plan_.begin();

        if(last_waypoint_)
        {
            utility::gl_vertex3f(last_waypoint_->get_position());
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

    for(auto &gsp:segments_)
    {
        gsp.second->draw();
    }

    components_mutex_.unlock();
}

void behaviour_planner::plan_sensing_tour(std::vector<grid_segment::ptr> &segments, const Vector3f &pos,
                                          const double &available_flight_time, const waypoint::ptr &cur_waypoint,
                                          plan &cur_plan)
{
    if(segments.empty())
        return;

    bool flag = true;

    cur_plan.push_front(cur_waypoint);

    waypoint::ptr curpos_waypoint = std::make_shared<waypoint>(pos);
    curpos_waypoint->set_action(waypoint::action::START_SENSING);
    cur_plan.push_front(curpos_waypoint);

    //move curpos_waypoint a little forward
    auto fp = mav_.sensor_.get_rect(pos);
    double step_forward = 0.5*std::min(fabs(fp[0]-fp[2]),fabs(fp[1]-fp[3]));
    bool has_curpos_wp = true;
    if(utility::distance_squared(pos, cur_waypoint->get_position()) > step_forward*step_forward)
        curpos_waypoint->set_position(pos+step_forward*(cur_waypoint->get_position()-pos).normalized());
    else
    {
        has_curpos_wp = false;
        cur_plan.pop_next_waypoint();
    }

    plan coverage_plan;

    Vector3f switching_pos = pos;

    while(flag)
    {
        double max_fac = 0;
        grid_segment::ptr max_fac_seg = nullptr;
        for(auto &seg: segments)
        {
            if(seg->is_selected())
                continue;

            double reaching_cost = seg->get_reaching_cost(switching_pos);
            double value = seg->get_segment_value();
            double fac =value/reaching_cost;

            //auto color = seg->get_color();

            ROS_INFO("val: %.1f r_cost: %.1f fac_v: %.1f l: %u", value, reaching_cost, fac, seg->get_label());

            if(max_fac < fac)
            {
                max_fac = fac;
                max_fac_seg = seg;
            }
        }


        if(max_fac_seg)
        {
            ROS_INFO("selected seg: %.1f reaching cost: %.1f label %d",
                     max_fac_seg->get_segment_value(), max_fac_seg->get_reaching_cost(switching_pos), max_fac_seg->get_label());

            size_t before_coverage_size = coverage_plan.size();
            flag = construct_coverage_plan(max_fac_seg, switching_pos, available_flight_time, cur_waypoint, cur_plan, coverage_plan);

            if(before_coverage_size != coverage_plan.size())
                max_fac_seg->set_selected(true);

            max_fac_seg->set_ignored();
        }
        else
        {
            ROS_INFO("No segment is selected");
            flag = false;
        }

        if(!coverage_plan.empty())
            switching_pos = coverage_plan.back()->get_position();
    }

    if(coverage_plan.empty())
    {
        cur_plan.pop_next_waypoint();
        if(has_curpos_wp)
            cur_plan.pop_next_waypoint();
    }
    else
    {
            while(!coverage_plan.empty())
            {
                coverage_plan.back()->set_waypoint_call_back([](waypoint::ptr wp_ptr)
                {
//                    ROS_INFO("WP CB: %f %f %f Coverage WP",
//                            wp_ptr->get_position()[0],
//                            wp_ptr->get_position()[1],
//                            wp_ptr->get_position()[2]);
                });

                cur_plan.push_front(coverage_plan.back());
                coverage_plan.pop_last_waypoint();
            }

            controller_action_ = waypoint::action::INTERRUPT_WAYPOINT;
    }
}

bool behaviour_planner::construct_coverage_plan(grid_segment::ptr segment, const Vector3f &pos,
                                                const double &available_flight_time, const waypoint::ptr &cur_waypoint,
                                                plan &cur_plan, plan &coverage_plan)
{
    std::vector<Vector3f> seg_coverage_path;
    segment->get_coverage_path(seg_coverage_path);

    if(seg_coverage_path.size() < 2)
        return false;

    if(utility::distance_squared(seg_coverage_path.front(), pos) >
            utility::distance_squared(seg_coverage_path.back(), pos))
        std::reverse(seg_coverage_path.begin(), seg_coverage_path.end());

    while(coverage_plan.cost(pos, cur_plan) < available_flight_time)
    {
        if(seg_coverage_path.empty())
            break;

        waypoint::ptr first_waypoint = std::make_shared<waypoint>(seg_coverage_path.front());
        seg_coverage_path.erase(seg_coverage_path.begin());

        first_waypoint->set_on_set_action(waypoint::action::STOP_SENSING);
        first_waypoint->set_type(waypoint::type::HIGH_RESOLUTION);

        coverage_plan.push_back(first_waypoint);
    }

    return seg_coverage_path.empty();
}

void behaviour_planner::greedy()
{
    update_grid_gp(covered_cells_.begin(), covered_cells_.end());
    update_segments(covered_cells_.begin(), covered_cells_.end());
    covered_cells_.clear();

    std::vector<grid_segment::ptr> segments;
    for(auto &sp: segments_)
    {
        auto seg = sp.second;
        seg->plan_coverage_path(2*active_survey_param::sensing_height,
                                active_survey_param::sensing_height);

        if(seg->is_valid() && !seg->get_ignored())
            segments.push_back(seg);
        else
        {
            ROS_INFO("invalid segment exists: l: %u value: %.1f coverage_path_size: %ld", seg->get_label(),
                     seg->get_segment_value(), seg->get_coverage_path_waypoint_count());
        }
    }

    //ROS_INFO(" before planning .....");
    plan_sensing_tour(segments, last_sensing_position_, get_available_time_(), last_waypoint_, plan_);
    //ROS_INFO(" after planning ......");

    for(auto seg:segments)
        seg->set_ignored();

    components_mutex_.lock();
    segments.clear();
    components_.clear();
    graph_->clear();
    components_mutex_.unlock();
}

void behaviour_planner::semi_greedy()
{

}

void behaviour_planner::two_stage()
{

}

}

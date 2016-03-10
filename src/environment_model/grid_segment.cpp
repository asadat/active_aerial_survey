#include <Eigen/Dense>

#include "environment_model/grid_segment.h"
#include "active_survey_param.h"
#include "math/graph.h"
#include "planner/trajectory_planner.h"

using namespace std;

namespace asn
{

grid_segment::grid_segment(grid &grd, grid_cell::ptr seed_cell, grid_cell_base::label label):
    graph_node(),
    grid_(grd),
    is_selected_(false),
    sudo_center_(0,0),
    cross_color_(0.9, 0.8, 0.0),
    delayed_segment_(nullptr)
{
    cells_.insert(seed_cell);
    seed_cell->set_label(label);
}

grid_segment::~grid_segment()
{
}

void grid_segment::reset_cell(grid_cell::ptr cell)
{
    cell->set_label(0);
    cell->set_approx_label(0);
    reset_visited(cell);
}

void grid_segment::clear()
{
    cells_.clear();
    boundary_cells_.clear();
    approximate_poly_cells_.clear();
    approximate_polygon_.clear();
    convexhull_.clear();
    coverage_path_.clear();
    uncertain_boundary_.clear();
}

void grid_segment::grow(std::function<grid_segment::ptr(grid_cell_base::label)> segments_accessor)
{
    vector<grid_cell::ptr> tovisit;
    tovisit.push_back(*cells_.begin());
    set_visited(tovisit.front());

    grid_cell_base::label label = tovisit.front()->get_label();

    while(!tovisit.empty())
    {
        grid_cell::ptr c = tovisit.back();
        tovisit.pop_back();

        c->set_label(label);
        cells_.insert(c);

        for(size_t i=0; i<4; i++)
        {
            auto nc = grid_.get_neighbour_cell_4(c,i);
            if(nc)
            {
                if(nc->has_label())
                {
                    if(nc->get_label() != label && !nc->is_ignored())
                    {
                        add_edge(std::static_pointer_cast<graph_node>(segments_accessor(label)),
                                 std::static_pointer_cast<graph_node>(segments_accessor(nc->get_label())));
                    }
                    continue;
                }

                if(nc->is_target() && !nc->is_ignored() && nc->is_covered() && !is_visited(nc))
                {
                    set_visited(nc);
                    tovisit.push_back(nc);
                }
            }
        }
    }
}

void grid_segment::merge_with_segment(ptr u, ptr merging_segment, graph::ptr &component)
{
    component->merge_nodes(std::static_pointer_cast<graph_node>(u),
                           std::static_pointer_cast<graph_node>(merging_segment));

    merging_segment->set_label(u->get_label());

    for(auto it=merging_segment->cells_.begin();
        it!=merging_segment->cells_.end();++it)
        u->cells_.insert(*it);

    merging_segment->clear();

    u->find_approximate_polygon();
    u->find_convexhull();
}

void grid_segment::set_label(grid_cell_base::label l)
{
    for(auto &c: cells_)
        c->set_label(l);
}

int grid_segment::get_approximate_neighbours_count(grid_cell::ptr cell)
{
    int n=0;
    for(int i=0; i<4; i++)
    {
        auto nb = grid_.get_neighbour_cell_4(cell, i);
        if(nb && nb->get_label()==cell->get_label()
                && nb->get_approx_label()==nb->get_label())
        {
            n++;
        }
    }
    return n;
}

void grid_segment::remove_skinny_part(grid_cell::ptr cell)
{
    if(!cell)
        return;

    if(cell->is_checked_for_skinny())
        return;

    cell->set_checked_for_skinny(true);

    for(int i=0; i<4; i+=1)
    {
        auto nb = grid_.get_neighbour_cell_4(cell, i);
        if(nb && nb->get_label() == cell->get_label()
                && nb->get_approx_label()==nb->get_label())
        {
            if(get_approximate_neighbours_count(nb) <= 1)
            {
                nb->set_approx_label(0);
                remove_skinny_part(nb);
            }
        }
    }
}

void grid_segment::find_approximate_polygon()
{
    sudo_center_ <<0,0;

    for(auto cell:cells_)
    {
        sudo_center_ += cell->get_center();
        cell->set_approx_label(cell->get_label());
        cell->set_checked_for_skinny(false);
    }

    if(!cells_.empty())
        sudo_center_ *= (1.0/cells_.size());

    // remove the parts with 1-cell width
    for(auto cell:cells_)
    {
        int nbs=0;
        for(int j=0; j<4; j++)
        {
            auto c = grid_.get_neighbour_cell_4(cell, j);
            if(c && c->get_label() == cell->get_label())
            {
                nbs++;
            }
        }

        if(nbs <= 1)
        {
            cell->set_approx_label(0);
            remove_skinny_part(cell);
        }
    }

    // find the boundary cells
    set<grid_cell::ptr> nds;
    for(auto nd:cells_)
    {
        if(nd->get_approx_label() != nd->get_label())
            continue;

        for(int i=0; i<4; i++)
        {
            auto c = grid_.get_neighbour_cell_4(nd, i);

            if(!c || (c->get_label()!=this->get_label()) || (c->get_approx_label() != nd->get_label()))
            {
                nds.insert(nd);
                break;
            }
        }
    }

    if(nds.empty())
        return;

    boundary_cells_.clear();
    copy(nds.begin(), nds.end(), back_inserter(boundary_cells_));
    get_uncertain_neighbour_cells(back_inserter(uncertain_boundary_));

    auto minit = std::min_element(nds.begin(), nds.end(), [](grid_cell::ptr x, grid_cell::ptr y)-> bool
    {
            grid_index xi,yi;
            xi = x->get_index();
            yi = y->get_index();
            if(xi[1] < yi[1]) return true;
    if(xi[1] > yi[1]) return false;
    return (xi[0] < yi[0]);
});

approximate_poly_cells_.clear();
approximate_poly_cells_.push_back(*minit);
nds.erase(minit);

// sort the boundary nodes
while(!nds.empty())
{
    auto cell = approximate_poly_cells_.back();
    size_t i=0;
    for(; i<8; i++)
    {
        auto nb = grid_.get_neighbour_cell_8(cell, (i+1)%8);
        if(!nb)
            continue;

        auto nb_it = nds.find(nb);
        if(nb_it != nds.end())
        {
            approximate_poly_cells_.push_back(*nb_it);
            nds.erase(nb_it);
            break;
        }
    }

    if(i>=8 && !nds.empty())
    {
        //ROS_ERROR("incomplete approximate polygon!!!!!!!!!!!!! %u c: %.2f %.2f %.2f", get_label(), get_color()[0], get_color()[1], get_color()[2]);
        break;
    }
}

//ROS_INFO("approx polygon for segment 1 ...");

//    std::vector<grid_cell::ptr> simplified_approx_polygon;
//    simplified_approx_polygon.push_back(approximate_poly_cells_.front());

//    for(auto it = approximate_poly_cells_.begin(); it!= approximate_poly_cells_.end();)
//    {
//        const double mean_dist_threshold = active_survey_param::min_footprint*0.5;
//        for(auto it_end = it+1; it_end != approximate_poly_cells_.end(); it_end++)
//        {
//            auto it_next = it_end+1;
//            if(it_next == approximate_poly_cells_.end())
//            {
//                simplified_approx_polygon.push_back(*it_end);
//                it = it_next;
//                break;
//            }
//            else
//            {
//                double mean_dist = 0;
//                double sz = 0;
//                for(auto it_tmp=it+1; it_tmp!=it_next; it_tmp++)
//                {
//                    sz+=1;
//                    mean_dist += utility::point_to_line_distance((*it)->get_center(), (*it_next)->get_center(), (*it_tmp)->get_center());
//                }

//                if(sz < 1.0)
//                    sz = 1.0;

//                mean_dist /= sz;

//                if(mean_dist > mean_dist_threshold)
//                {
//                    //ROS_INFO("************ mean_dist: %f", mean_dist);
//                    it = it_end;
//                    simplified_approx_polygon.push_back(*it_end);
//                    break;
//                }
//            }
//        }
//    }

//    approximate_poly_cells_.clear();
//    copy(simplified_approx_polygon.begin(), simplified_approx_polygon.end(), back_inserter(approximate_poly_cells_));

approximate_polygon_.clear();
for(auto it=approximate_poly_cells_.begin(); it !=approximate_poly_cells_.end(); it++)
approximate_polygon_.push_back((*it)->get_center());
}

void grid_segment::find_convexhull()
{
    polygon_convexhull::find_convexhull(approximate_polygon_, convexhull_, is_line_);
}

//void grid_segment::find_convexhull()
//{
//    //ROS_INFO("Convexhull l: %u #approx: %ld", get_label(), approximate_polygon_.size());

//    convexhull_.clear();
//    if(approximate_polygon_.size()<3)
//    {
//        //ROS_ERROR("CH l: %u approx_poly.size() < 3", get_label());
//        return;
//    }

//    // first node is the bottom most node
//    auto first = std::min_element(approximate_polygon_.begin(), approximate_polygon_.end(),
//                                  [](const Vector2f &v, const Vector2f &u){return v[1]<u[1];});

//    convexhull_.push_back(*first);


//    //second node
//    const Vector2f &p2 = convexhull_.front();
//    const Vector2f &p1 = p2 - Vector2f(-1, 0);

//    auto second = std::max_element(approximate_polygon_.begin(), approximate_polygon_.end(),
//                                   [&p1, &p2](const Vector2f &v, const Vector2f &u)
//    {
//        if(utility::close_enough(p2,u))
//            return false;
//        else if(utility::close_enough(p2,v))
//            return true;

//        return utility::angle(p1, p2, v) < utility::angle(p1, p2, u);
//    });

//    convexhull_.push_back(*second);

//    //check if the cells form a line a line
//    is_line_ = true;
//    const Vector2f &p_0 = approximate_polygon_.front();
//    const Vector2f &p_1 = approximate_polygon_.back();

//    for(auto it=approximate_polygon_.begin()+2; it!=approximate_polygon_.end(); it++)
//    {
//        auto &p_n = *it;

//        if(fabs((p_0[1]-p_1[1])*(p_0[0]-p_n[0]) - (p_0[1]-p_n[1])*(p_0[0]-p_1[0])) > 0.01)
//        {
//            is_line_ = false;
//            break;
//        }
//    }

//    if(is_line_)
//    {
//        //ROS_INFO("Convex hull detected line.");
//        convexhull_.clear();
//        std::copy(approximate_polygon_.begin(), approximate_polygon_.end(), std::back_inserter(convexhull_));
//    }
//    else
//    {
//        auto approximate_polygon = approximate_polygon_;
//        bool added_new_vertex = true;
//        // the rest of the nodes
//        while(added_new_vertex)
//        {
//            const auto &p1 = *(--(--convexhull_.end()));
//            const auto &p2 = convexhull_.back();

//            const auto next = std::max_element(approximate_polygon.begin(), approximate_polygon.end(),
//                                               [&p1, &p2](const Vector2f &v, const Vector2f &u)
//            {
//                //                  if(utility::close_enough(p2,u))
//                //                  {
//                //                      ROS_INFO("CH compare!!!");
//                //                      return false;
//                //                  }
//                //                  else if(utility::close_enough(p2,v))
//                //                  {
//                //                      ROS_INFO("CH compare!!!");
//                //                      return true;
//                //                  }

//                return utility::angle(p1, p2, v) < utility::angle(p1, p2, u);
//            });

//            if(next != approximate_polygon.end())
//            {

//                //ROS_INFO("CH angle: %.2f %f %f %f %f %f %f", utility::angle(p1, p2, *next), p1[0], p1[1], p2[0], p2[1], (*next)[0], (*next)[1]);
//            }
//            else
//                ROS_INFO("CH angle: No min");

//            if(next != approximate_polygon.end())
//            {
//                //if(std::find(convexhull_.begin(), convexhull_.end(), *next) != convexhull_.end())
//                //   added_new_vertex=false;
//                //else
//                //    convexhull_.push_back(*next);

//                if(utility::close_enough(convexhull_.front(), *next))
//                    added_new_vertex = false;
//                else
//                    convexhull_.push_back(*next);
//            }
//            else
//            {
//                added_new_vertex = false;
//            }
//        }

//        //ROS_INFO("Convexhull l: %u #before_simplification: %ld", get_label(), convexhull_.size());
//        if(convexhull_.size() == 2)
//        {
//            //            ROS_WARN("P1: %.1f %.1f P2: %.1f %.1f",
//            //                     convexhull_[0][0], convexhull_[0][1],
//            //                     convexhull_[1][0], convexhull_[1][1]);
//        }
//        else
//        {
//            // remove extra nodes (colinear edges)
//            size_t sz = convexhull_.size();

//            for(int i=0; i < (int)convexhull_.size(); i++)
//            {
//                sz = convexhull_.size();

//                int i1 = ((i-1)+sz)%sz;
//                int i2 = i;
//                int i3 = (i+1)%sz;

//                Vector2f v = convexhull_[i2] - convexhull_[i1];
//                Vector2f u = convexhull_[i3] - convexhull_[i2];

//                Vector3f v3(v[0], v[1], 0.0);
//                Vector3f u3(u[0], u[1], 0.0);

//                auto r = u3.cross(v3);

//                if(sqrt(r.dot(r)) < 0.1)
//                {
//                    convexhull_.erase(convexhull_.begin()+i);
//                    i--;
//                }
//            }
//        }
//    }
//    //std::reverse(convexhull_.begin(), convexhull_.end());
//    //ROS_INFO("Convexhull l: %u #ch: %ld", get_label(), convexhull_.size());
//}

//bool grid_segment::find_base_edge(size_t &first_index, size_t &second_index, double &convexhull_height) const
//{
//    if(!is_valid())
//        return false;

//    if(is_line_)
//    {
//        double max_dist = 0;
//        for(size_t i=0; i<convexhull_.size(); ++i)
//            for(size_t j=0; j<convexhull_.size(); ++j)
//            {
//                double dist = utility::distance_squared(convexhull_[i], convexhull_[j]);
//                if(dist > max_dist)
//                {
//                    max_dist = dist;
//                    first_index = i;
//                    second_index = j;
//                }
//            }

//        return true;
//    }

//    size_t size = convexhull_.size();
//    double min_height = 999999;
//    size_t min_height_index = 0;

//    for(size_t i=0; i < size; i++)
//    {
//        double max_height = 0;

//        for(size_t j=0; j < size; j++)
//        {
//            if(j==i || (i+1)%size ==j)
//                continue;

//            double h = utility::point_to_line_distance(convexhull_[i],
//                                                       convexhull_[(i+1)%size],
//                    convexhull_[j]);

//            max_height = (max_height < h) ? h : max_height;
//        }

//        if(min_height > max_height)
//        {
//            min_height = max_height;
//            convexhull_height = min_height;
//            min_height_index = i;
//        }
//    }

//    double sd = utility::point_to_line_signed_distance(convexhull_[min_height_index],
//                                                       convexhull_[(min_height_index+1)%size],
//            convexhull_[(min_height_index+2)%size]);
//    if(sd > 0)
//    {
//        first_index = min_height_index;
//        second_index = (min_height_index+1)%size;
//    }
//    else
//    {
//        second_index = min_height_index;
//        first_index = (min_height_index+1)%size;
//    }

//    return true;
//}

bool grid_segment::plan_coverage_path(const double &inter_lap_distance, const double &altitude)
{
    return trajectory_planner::plan_coverage_path(convexhull_, is_line_, inter_lap_distance, altitude, coverage_path_, coverage_path_cost_);
}

//bool grid_segment::plan_coverage_path(const double &inter_lap_distance, const double &altitude)
//{
//    coverage_path_.clear();
//    coverage_path_cost_=0;

//    vector<Vector2f> coverage_path;

//    size_t base[2];
//    double convexhull_height=0;
//    if(!find_base_edge(base[0], base[1], convexhull_height))
//        return false;

//    if(is_line_)
//    {
//        coverage_path_.push_back(utility::augment(convexhull_[base[0]], altitude));
//        coverage_path_.push_back(utility::augment(convexhull_[base[1]], altitude));
//        coverage_path_cost_ = utility::distance(coverage_path_.front(), coverage_path_.back());
//        return true;
//    }

//    Vector2f base_dir  = convexhull_[base[1]]-convexhull_[base[0]];
//    Vector2f base_dir_norm = base_dir.normalized();

//    //clockwise orthogonal vector
//    Vector2f sweep_dir(base_dir_norm[1], -base_dir_norm[0]);

//    Vector2f sn0 = convexhull_[base[0]];
//    Vector2f en0 = convexhull_[base[1]];
//    double offset0 = inter_lap_distance*0.5;

//    sn0 += offset0 * sweep_dir;
//    en0 += offset0 * sweep_dir;

//    // first lm track
//    coverage_path.push_back(sn0);
//    coverage_path.push_back(en0);

//    double n =-1;
//    while(n <= ceil((convexhull_height+ 2*active_survey_param::sensing_height) / inter_lap_distance))
//    {
//        n+=1.0;

//        Vector2f sn = sn0;
//        Vector2f en = en0;
//        sn += n * inter_lap_distance * sweep_dir;
//        en += n * inter_lap_distance * sweep_dir;

//        sn -=  2 * active_survey_param::area_width * base_dir_norm;
//        en +=  2 * active_survey_param::area_width * base_dir_norm;

//        vector<Vector2f> intersections;

//        for(size_t i=0; i< convexhull_.size(); i++)
//        {
//            Vector2f ise(0,0);
//            if(utility::get_line_segment_intersection(sn, en, convexhull_[i], convexhull_[(i+1)%(convexhull_.size())], ise))
//            {
//                intersections.push_back(ise);
//            }
//        }

//        if(intersections.size() > 1)
//        {
//            if(intersections.size() > 2)
//            {
//                for(size_t i=0; i<intersections.size(); i++)
//                {
//                    for(size_t j=i+1; j<intersections.size(); j++)
//                    {
//                        if(utility::distance_squared(intersections[i],intersections[j]) < 0.1)
//                        {
//                            intersections.erase(intersections.begin()+j);
//                            break;
//                        }
//                    }
//                }
//            }

//            if(intersections.size()>=2)
//            {
//                if(utility::distance_squared(intersections[0],coverage_path[coverage_path.size()-2]) <
//                        utility::distance_squared(intersections[1],coverage_path[coverage_path.size()-2]))
//                {
//                    coverage_path.push_back(intersections[1]);
//                    coverage_path.push_back(intersections[0]);
//                }
//                else
//                {
//                    coverage_path.push_back(intersections[0]);
//                    coverage_path.push_back(intersections[1]);
//                }
//            }
//        }
//        else if(intersections.size() == 1)
//        {

//        }
//        else
//        {
//            // no intersection !!!!!!
//        }
//    }

//    if(coverage_path.size() > 2)
//    {
//        for(auto &p: coverage_path)
//        {
//            p -= 0.3*(offset0) * sweep_dir;
//        }
//        coverage_path.erase(coverage_path.begin());
//        coverage_path.erase(coverage_path.begin());
//    }
//    else
//    {
//        coverage_path[0] -= ((offset0)-convexhull_height/2) * sweep_dir;
//        coverage_path[1] -= ((offset0)-convexhull_height/2) * sweep_dir;
//    }



//    for(auto it=coverage_path.begin(), prev=it; it!=coverage_path.end(); ++it)
//    {
//        coverage_path_cost_ += utility::distance(*it, *prev);
//        prev = it;
//        coverage_path_.push_back(utility::augment(*it, altitude));
//    }

//    coverage_path_cost_ += (coverage_path_.size()-2)* active_survey_param::turning_time;

//    return true;
//}

double grid_segment::get_coverage_path_cost(const Vector3f &from) const
{
    return get_coverage_path_switching_cost(from) + coverage_path_cost_;
}

double grid_segment::get_coverage_path_switching_cost(const Vector3f &from) const
{
    double switching_cost = utility::distance(from, coverage_path_.front()) +
            utility::distance(from, coverage_path_.back())+ 2.0*active_survey_param::turning_time;
    return switching_cost;
}

double grid_segment::get_reaching_cost(const Vector3f &from) const
{
    double reaching_cost_a = utility::distance(from, coverage_path_.front())
            + active_survey_param::turning_time;

    double reaching_cost_b = utility::distance(from, coverage_path_.back())
            + active_survey_param::turning_time;

    return std::min(reaching_cost_a, reaching_cost_b);
}

double grid_segment::get_segment_value() const
{
    if(!cells_.empty())
        return cells_.size() * (*begin())->get_area();
    else
        return 0.0;
}

void grid_segment::get_coverage_path(std::vector<Vector3f> &coverage_path) const
{
    for(auto it=coverage_path_.begin(); it !=coverage_path_.end(); ++it)
        coverage_path.push_back(*it);
}

void grid_segment::set_ignored()
{
    for(auto it = begin(); it != end();++it)
        (*it)->set_ignored(true);
}

template<class OutIterator>
void grid_segment::get_uncertain_neighbour_cells(OutIterator out_iterator)
{
    std::set<grid_cell::ptr> uncertain_cells;
    for(auto &bn: boundary_cells_)
    {
        for(size_t i=0; i<4; i++)
        {
            auto nc = grid_.get_neighbour_cell_4(bn, i);
            if(nc)
            {
                if(/*nc->is_uncertain() ||*/ !nc->is_covered())
                {
                    uncertain_cells.insert(nc);
                }
            }
        }
    }

    copy(uncertain_cells.begin(), uncertain_cells.end(), out_iterator);
}

//template<class OutIterator>
//void grid_segment::find_uncertain_neighbour_cells(OutIterator out_iterator)
//{
//    std::set<grid_cell::ptr> uncertain_cells;
//    for(auto &bn: boundary_cells_)
//    {
//        for(size_t i=0; i<4; i++)
//        {
//            auto nc = grid_.get_neighbour_cell_4(bn, i);
//            if(nc)
//            {
//                if(nc->is_uncertain() && !nc->is_covered())
//                {
//                    uncertain_cells.insert(nc);
//                }
//            }
//        }
//    }

//    copy(uncertain_cells.begin(), uncertain_cells.end(), out_iterator);
//}

double grid_segment::get_uncertain_neighbour_area() const
{
    auto s = grid_.get_cell_size();
    return uncertain_boundary_.size() * s[0] * s[1];
}

bool grid_segment::is_uncertain() const
{
    return get_uncertain_neighbour_area() >
            active_survey_param::uncertain_cells_threshold;
}

void grid_segment::draw()
{
    glLineWidth(2);
    utility::gl_color(get_color());

    if(true || active_survey_param::non_ros::cell_drawing_mode==5)
    {
        if(convexhull_.size() >= 3)
        {
//            double dc = 1;
//            if(!convexhull_.empty())
//                dc = 1.0/convexhull_.size();
            double i = 0;

            glBegin(GL_LINES);
            for(auto it=begin_convexhull(); it!=end_convexhull(); ++it)
            {
                //glColor3f(i*dc, 0, 1);
                utility::gl_vertex3f(*it, 0.3);
                utility::gl_vertex3f(*(it+1!=end_convexhull()?it+1:begin_convexhull()), 0.3);
                i+=1.0;
            }
            glEnd();
        }
    }


    //    glPointSize(4);
    //    glBegin(GL_POINTS);
    //    for(auto cell:boundary_cells_)
    //        utility::gl_vertex3f(cell->get_center(), 0.4);
    //    glEnd();

//    glColor4f(1, 1,1, 0.8);
//    glPointSize(10);
//    glBegin(GL_POINTS);
//    for(auto cell:uncertain_boundary_)
//        utility::gl_vertex3f(cell->get_center(), 0.4);
//    glEnd();

        if(is_valid())
        {

//            utility::gl_color(cross_color_);
//            if(is_uncertain())
//                glLineWidth(8);
//            else
//                glLineWidth(3);

//            glBegin(GL_LINES);
//            utility::draw_cross(sudo_center_, 0.5);
//            glEnd();

//            if(delayed_segment_)
//            {
//                glColor3f(0.5, 0.5,1);
//                glLineWidth(2);
//                glBegin(GL_LINES);
//                utility::gl_vertex3f(delayed_segment_->get_sudo_center(), 0.51);
//                utility::gl_vertex3f(sudo_center_, 0.51);
//                glEnd();
//            }

//            if(is_uncertain())
//            {
//                utility::gl_color(cross_color_);
//                glLineWidth(1);
//                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//                glBegin(GL_POLYGON);
//                utility::draw_circle(sudo_center_, 2*active_survey_param::coarse_coverage_height, 0.52);
//                glEnd();
//            }
        }

//        if(is_valid())
//        {
//            glColor3f(0,1,0);
//            glLineWidth(2);
//            glBegin(GL_LINES);
//            for(size_t i=0; i+1<coverage_path_.size();i++)
//            {
//                utility::gl_vertex3f(coverage_path_[i]);
//                utility::gl_vertex3f(coverage_path_[i+1]);
//                //glColor3f(1,0,0);
//            }
//            glEnd();

//	    glColor3f(1,1,0);
//            glPointSize(3);
//            glBegin(GL_POINTS);
//            for(size_t i=0; i<coverage_path_.size();i++)
//            {
//                utility::gl_vertex3f(coverage_path_[i]+Vector3f(0,0,0.1));
//            }
//            glEnd();
//        }

    //    glPointSize(6);
    //    glBegin(GL_POINTS);

    //    dc=1;
    //    if(!convexhull_.empty())
    //        dc = 1.0/convexhull_.size();
    //    i=1.0;

    //    for(auto it=begin_convexhull(); it!= end_convexhull(); it++)
    //    {
    //        glColor3f(i*dc, 1,0);
    //        utility::gl_vertex3f(*it, 0.3);
    //        i+=1.0;
    //    }

    //    glEnd();
}

}

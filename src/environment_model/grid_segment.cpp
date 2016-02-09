#include <Eigen/Dense>

#include "environment_model/grid_segment.h"
#include "active_survey_param.h"

using namespace std;

namespace asn
{

grid_segment::grid_segment(grid &grd, grid_cell::ptr seed_cell, grid_cell_base::label label):
    graph_node(),
    grid_(grd)
{
    cells_.insert(seed_cell);
    seed_cell->set_label(label);
}

grid_segment::~grid_segment()
{}

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
                    if(nc->get_label() != label)
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
    for(auto cell:cells_)
    {
        cell->set_approx_label(cell->get_label());
        cell->set_checked_for_skinny(false);
    }

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

    while(!nds.empty())
    {
        auto cell = approximate_poly_cells_.back();
        size_t i=0;
        for(; i<8; i++)
        {
            auto nb = grid_.get_neighbour_cell_8(cell, i);
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

        if(i>=8)
        {
            //ROS_ERROR("incomplete approximate polygon!!!!!!!!!!!!! %d %d", node->grd_x, node->grd_y);
            break;
        }
    }

    std::vector<grid_cell::ptr> simplified_approx_polygon;
    simplified_approx_polygon.push_back(approximate_poly_cells_.front());

    for(auto it = approximate_poly_cells_.begin(); it!= approximate_poly_cells_.end();)
    {
        const double mean_dist_threshold = active_survey_param::min_footprint*0.5;
        for(auto it_end = it+1; it_end != approximate_poly_cells_.end(); it_end++)
        {
            auto it_next = it_end+1;
            if(it_next == approximate_poly_cells_.end())
            {
                simplified_approx_polygon.push_back(*it_end);
                it = it_next;
                break;
            }
            else
            {
                double mean_dist = 0;
                double sz = 0;
                for(auto it_tmp=it+1; it_tmp!=it_next; it_tmp++)
                {
                    sz+=1;
                    mean_dist += utility::point_to_line_distance((*it)->get_center(), (*it_next)->get_center(), (*it_tmp)->get_center());
                }

                if(sz < 1.0)
                    sz = 1.0;

                mean_dist /= sz;

                if(mean_dist > mean_dist_threshold)
                {
                    //ROS_INFO("************ mean_dist: %f", mean_dist);
                    it = it_end;
                    simplified_approx_polygon.push_back(*it_end);
                    break;
                }
            }
        }
    }

    approximate_poly_cells_.clear();
    copy(simplified_approx_polygon.begin(), simplified_approx_polygon.end(), back_inserter(approximate_poly_cells_));

    approximate_polygon_.clear();
    for(auto it=approximate_poly_cells_.begin(); it !=approximate_poly_cells_.end(); it++)
      approximate_polygon_.push_back((*it)->get_center());
}

void grid_segment::find_convexhull()
{
    convexhull_.clear();
    if(approximate_polygon_.size()<3)
    {
        return;
    }

    // first node is the bottom most node
    size_t first=0;
    for(unsigned int i=1; i<approximate_polygon_.size(); i++)
        if(approximate_polygon_[first][1]> approximate_polygon_[i][1])
            first = i;

    convexhull_.push_back(approximate_polygon_[first]);


    //second node
    size_t second=1;
    Vector2f p1 = convexhull_[0] - Vector2f(-1, 0);
    Vector2f p2 = convexhull_.back();

    double ang = utility::angle(p1, p2, approximate_polygon_[1]);
    for(size_t i=2; i<approximate_polygon_.size(); i++)
    {
        double a = utility::angle(p1, p2, approximate_polygon_[i]);
        if(a> ang)
        {
            ang = a;
            second = i;
        }
    }

    convexhull_.push_back(approximate_polygon_[second]);

    //check if the cells form a line a line
    bool is_line = true;
    Vector2f p_0 = approximate_polygon_[0];
    Vector2f p_1 = approximate_polygon_[1];

    for(size_t i=2; i < approximate_polygon_.size() && is_line; i++)
    {
        Vector2f p_n = approximate_polygon_[i];
        if(fabs((p_0[1]-p_1[1])*(p_0[0]-p_n[0]) - (p_0[1]-p_n[1])*(p_0[0]-p_1[0])) > 0.01)
        {
            is_line = false;
        }
    }

    if(is_line)
    {
        convexhull_.clear();
        std::copy(approximate_polygon_.begin(), approximate_polygon_.end(), std::back_inserter(convexhull_));
    }
    else
    {
        // the rest of the nodes
        while(true)
        {
            //if(n++ > 100) return;
            p1 = convexhull_[convexhull_.size()-2];
            p2 = convexhull_.back();
            int next = -1;
            ang = -1;
            for(size_t i=0; i<approximate_polygon_.size(); i++)
            {
                if(approximate_polygon_[i] == convexhull_.back())
                    continue;

                double a = utility::angle(p1, p2, approximate_polygon_[i]);
                if(a> ang)
                {
                    ang = a;
                    next = i;
                }
            }

            if(next > -1)
            {
                if(std::find(convexhull_.begin(), convexhull_.end(), approximate_polygon_[next]) != convexhull_.end())
                    break;
                else
                    convexhull_.push_back(approximate_polygon_[next]);
            }
            else
            {
                break;
            }
        }

        // remove extra nodes (colinear edges)
        size_t sz = convexhull_.size();

        for(int i=0; i < (int)convexhull_.size(); i++)
        {
            sz = convexhull_.size();

            int i1 = ((i-1)+sz)%sz;
            int i2 = i;
            int i3 = (i+1)%sz;

            Vector2f v = convexhull_[i2] - convexhull_[i1];
            Vector2f u = convexhull_[i3] - convexhull_[i2];

            Vector3f v3(v[0], v[1], 0.0);
            Vector3f u3(u[0], u[1], 0.0);

            auto r = u3.cross(v3);

            if(sqrt(r.dot(r)) < 0.1)
            {
                convexhull_.erase(convexhull_.begin()+i);
                i--;
            }
        }
    }


//    center = makeVector(0,0,0);
//    for(size_t i=0; i < convexhull_.size(); i++)
//    {
//        center += convexhull_[i];
//    }

//    center = (1.0/(float)convexhull_.size())*center;

    std::reverse(convexhull_.begin(), convexhull_.end());
}

}

#pragma once

#include "environment_model/grid.h"
#include "math/graph_node.h"
#include "math/polygon.h"

#include <set>
#include <functional>


namespace asn
{

class grid_segment: public graph_node
{
public:
    typedef std::shared_ptr<grid_segment> ptr;
    typedef std::set<grid_cell::ptr>::const_iterator const_cell_iterator;
    typedef std::set<grid_cell::ptr>::iterator cell_iterator;
    typedef polygon::const_iterator const_polygon_vertex_iterator;

public:
    grid_segment(grid &grd, grid_cell::ptr seed_cell, grid_cell_base::label label);
    virtual ~grid_segment();

    void grow(std::function<grid_segment::ptr(grid_cell_base::label)> segments_accessor);

    void find_approximate_polygon();
    void find_convexhull();

    void get_approximate_polygon(polygon &approx_poly) const;
    void get_convexhull(polygon &convexhull) const;

    inline const_cell_iterator begin() const {return cells_.begin();}
    inline const_cell_iterator end() const {return cells_.end();}

    inline const_polygon_vertex_iterator begin_approx_poly() const {return approximate_polygon_.begin();}
    inline const_polygon_vertex_iterator end_approx_poly() const {return approximate_polygon_.end();}

    static inline bool is_visited(grid_cell::ptr &cell){return cell->flags_&1;}
    static inline void set_visited(grid_cell::ptr &cell){cell->flags_ |= 1;}

    virtual Vector2f get_position(){return (*begin())->get_center();}

    grid_cell_base::label get_label() const {return (*begin())->get_label();}

private:
    grid_segment()=delete;
    void remove_skinny_part(grid_cell::ptr cell);
    int get_approximate_neighbours_count(grid_cell::ptr cell);

    grid &grid_;
    std::set<grid_cell::ptr> cells_;
    std::vector<grid_cell::ptr> boundary_cells_;
    std::vector<grid_cell::ptr> approximate_poly_cells_;

    polygon approximate_polygon_;
    polygon convexhull_;


};

}

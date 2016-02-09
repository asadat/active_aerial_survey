#pragma once

#include "environment_model/grid.h"
#include "math/graph_node.h"

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

public:
    grid_segment(grid &grd, grid_cell::ptr seed_cell, grid_cell_base::label label);
    virtual ~grid_segment();

    void grow(std::function<grid_segment::ptr(grid_cell_base::label)> segments_accessor);
    inline const_cell_iterator begin() const {return cells_.begin();}
    inline const_cell_iterator end() const {return cells_.end();}

    static inline bool is_visited(grid_cell::ptr &cell){return cell->flags_&1;}
    static inline void set_visited(grid_cell::ptr &cell){cell->flags_ |= 1;}

    virtual Vector2f get_position(){return (*begin())->get_center();}

    grid_cell_base::label get_label() const {return (*begin())->get_label();}

private:
    grid_segment()=delete;

    grid &grid_;
    std::set<grid_cell::ptr> cells_;
};

}

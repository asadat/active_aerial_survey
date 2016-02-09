#pragma once

#include "environment_model/grid.h"
#include <set>
#include <functional>

namespace asn
{

class grid_segment
{
public:
    typedef std::shared_ptr<grid_segment> ptr;
    typedef std::set<grid_cell::ptr>::const_iterator const_cell_iterator;
    typedef std::set<grid_cell::ptr>::iterator cell_iterator;
    typedef std::set<grid_cell_base::label>::const_iterator const_neighbour_iterator;
    typedef std::set<grid_cell_base::label>::iterator neighbour_iterator;

public:
    grid_segment(grid &grd, grid_cell::ptr seed_cell, grid_cell_base::label label);
    virtual ~grid_segment();

    void grow(std::function<grid_segment::ptr(grid_cell_base::label)> segments_accessor);
    inline const_cell_iterator begin(){return cells_.begin();}
    inline const_cell_iterator end(){return cells_.end();}

    inline neighbour_iterator begin_neighbour(){return neighbours_.begin();}
    inline neighbour_iterator end_neighbour(){return neighbours_.end();}

    static inline bool is_visited(grid_cell::ptr &cell){return cell->flags_&1;}
    static inline void set_visited(grid_cell::ptr &cell){cell->flags_ |= 1;}

private:
    grid_segment()=delete;
    std::set<grid_cell_base::label> neighbours_;

    grid &grid_;
    std::set<grid_cell::ptr> cells_;

};
}

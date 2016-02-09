#pragma once

#include "environment_model/grid.h"
#include <set>

namespace asn
{

class grid_segment
{
public:
    typedef std::shared_ptr<grid_segment> ptr;
    typedef std::set<grid_cell::ptr>::const_iterator const_cell_iterator;
    typedef std::set<grid_cell::ptr>::iterator cell_iterator;

public:
    grid_segment(grid &grd, grid_cell::ptr seed_cell, grid_cell_base::label label);
    virtual ~grid_segment();

    void grow();
    inline const_cell_iterator begin(){return cells.begin();}
    inline const_cell_iterator end(){return cells.end();}

    static inline bool is_visited(grid_cell::ptr &cell){return cell->flags_&1;}
    static inline void set_visited(grid_cell::ptr &cell){cell->flags_ |= 1;}

private:
    grid_segment()=delete;

    grid &grid_;
    std::set<grid_cell::ptr> cells;

};
}

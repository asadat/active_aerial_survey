#include "environment_model/grid_segment.h"

using namespace std;

namespace asn
{

grid_segment::grid_segment(grid &grd, grid_cell::ptr seed_cell, grid_cell_base::label label):
    grid_(grd)
{
    cells.insert(seed_cell);
    seed_cell->set_label(label);
}

grid_segment::~grid_segment()
{}

void grid_segment::grow()
{
    vector<grid_cell::ptr> tovisit;
    tovisit.push_back(*cells.begin());
    set_visited(tovisit.front());

    grid_cell_base::label label = tovisit.front()->get_label();

    while(!tovisit.empty())
    {
        grid_cell::ptr c = tovisit.back();
        tovisit.pop_back();

        c->set_label(label);

        for(size_t i=0; i<4; i++)
        {
            auto nc = grid_.get_neighbour_cell_4(c,i);
            if(nc)
            {
                if(nc->is_target() && !nc->is_ignored() && nc->is_covered() && !nc->has_label() && !is_visited(nc))
                {
                    set_visited(nc);
                    tovisit.push_back(nc);
                }
            }
        }
    }
}

}
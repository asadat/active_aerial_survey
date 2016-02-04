#include "grid.h"
#include "ros/ros.h"
namespace asn
{

grid::grid(){}

grid::grid(const Vector2f &center, const size2f &cell_size, const size2i &size):
    center_(center),
    cell_size_(cell_size),
    size_(size)
{
    neighbours_idx_4 = {{0,-1},{1,0},{0,1},{-1,0}};
    neighbours_idx_8 = {{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0}};

    Vector2f ll = center - Vector2f(0.5*size_[0]*cell_size_[0], 0.5*size_[1]*cell_size_[1]);
    ll += Vector2f(0.5*cell_size_[0], 0.5*cell_size_[1]);

    for(int j=0; j<size_[1]; j++)
        for(int i=0; i<size_[0]; i++)
        {
            auto cell = grid_cell::ptr(new grid_cell(ll+ size2f(i*cell_size_[0], j*cell_size_[1]), cell_size_, grid_index(i,j)));
            cells.push_back(cell);
        }
}

grid::~grid()
{

}

grid_cell::ptr grid::get_cell(const grid_index &idx) const
{
    if(valid_index(idx))
    {
        return cells[idx[1]*size_[0]+idx[0]];
    }
    else
    {
        return nullptr;
    }
}

grid_cell::ptr grid::get_neighbour_cell(const grid_cell::ptr &ref, const grid_index &rel_idx) const
{
    if(!ref)
        return nullptr;

    grid_index idx = ref->get_index() + rel_idx;

    if(valid_index(idx))
        return get_cell(idx);
    else
        return nullptr;
}

void grid::draw()
{
    for(auto &c:cells)
        c->draw();
}

}

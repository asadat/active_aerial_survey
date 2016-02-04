#include "grid.h"

namespace asn
{

grid::grid(){}

grid::grid(const Vector2f &center, const size2f &cell_size, const size2i &size):
    center_(center),
    cell_size_(cell_size),
    size_(size)
{
    Vector2f ll = center - Vector2f(0.5*size_[0]*cell_size_[0], 0.5*size_[1]*cell_size_[1]);
    ll += Vector2f(0.5*cell_size_[0], 0.5*cell_size_[1]);

    for(int i=0; i<size_[0]; i++)
        for(int j=0; j<size_[1]; j++)
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
        return cells[idx[0]*size_[0]+idx[1]];
    }
    else
    {
        return nullptr;
    }
}

void grid::draw()
{
    for(auto &c:cells)
        c->draw();
}

}

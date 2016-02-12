#include "environment_model/grid.h"
#include "ros/ros.h"
namespace asn
{

grid::grid(){}

grid::grid(const Vector2f &center, const size2f &cell_size, const size2i &size):
    center_(center),
    cell_size_(cell_size),
    size_(size)
{
    cells_.reserve(size_[0]*size_[1]);

    neighbours_idx_4_ = {{0,-1},{1,0},{0,1},{-1,0}};
    neighbours_idx_8_ = {{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0}};

    Vector2f ll = center - Vector2f(0.5*size_[0]*cell_size_[0], 0.5*size_[1]*cell_size_[1]);
    ll += Vector2f(0.5*cell_size_[0], 0.5*cell_size_[1]);

    for(int j=0; j<size_[1]; j++)
        for(int i=0; i<size_[0]; i++)
        {
            //auto cell = grid_cell::ptr(new grid_cell(ll+ size2f(i*cell_size_[0], j*cell_size_[1]), cell_size_, grid_index(i,j)));
            auto cell = std::make_shared<grid_cell>(ll+ size2f(i*cell_size_[0], j*cell_size_[1]), cell_size_, grid_index(i,j));
            cells_.push_back(cell);
        }
}

grid::~grid()
{

}

grid_cell::ptr grid::get_cell(const grid_index &idx) const
{
    if(valid_index(idx))
    {
        return cells_[idx[1]*size_[0]+idx[0]];
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
    for(auto &c:cells_)
        c->draw();

    rect bbox(center_[0]-0.5*size_[0]*cell_size_[0], center_[1]-0.5*size_[1]*cell_size_[1],
            center_[0]+0.5*size_[0]*cell_size_[0], center_[1]+0.5*size_[1]*cell_size_[1]);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(2);
    glColor3f(0.3,0.6,0.8);

    glBegin(GL_QUADS);
    utility::draw_quad(bbox);
    glEnd();
}

grid_cell::ptr grid::find_cell_contains(const Vector2f &v) const
{
    if(cells_.empty())
        return nullptr;

    auto ll = get_cell({0,0})->get_corner_ll();
    grid_index idx = {floor((v[0]-ll[0])/cell_size_[0]), floor((v[1]-ll[1])/cell_size_[1])};
    return get_cell(idx);
}

void grid::find_cells_in_rect(const rect &range, std::set<grid_cell::ptr> &cells, bool completely_inside) const
{
    Vector2f v(range[0], range[1]);

    for(;v[0]<range[2]+cell_size_[0]; v[0]+=cell_size_[0])
    {
        v[1] = range[1];
        for(;v[1]<range[3]+cell_size_[1]; v[1]+=cell_size_[1])
        {
            auto c = find_cell_contains(v);
            if(c)
            {
                if(!completely_inside || utility::is_rect_inside_rect(c->get_rect(), range))
                {
                    cells.insert(c);
                }
            }
        }
    }

}

}

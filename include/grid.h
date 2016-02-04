#pragma once

#include <grid_cell.h>
#include <vector>

using namespace Eigen;

namespace asn
{

typedef Vector2i size2i;

class grid
{
public:
    grid(const Vector2f &center, const size2f &cell_size, const size2i &size);

    ~grid();

    inline size2f get_cell_size() const {return cell_size_;}
    inline Vector2f get_center() const {return center_;}
    inline size2i get_grid_size() const {return size_;}

    inline std::vector<grid_cell::ptr>::iterator begin(){return cells.begin();}
    inline std::vector<grid_cell::ptr>::iterator end(){return cells.end();}

    grid_cell::ptr get_cell(const grid_index &idx) const;
    grid_cell::ptr get_neighbour_cell(const grid_cell::ptr &ref, const grid_index &rel_idx) const;

    grid_cell::ptr get_neighbour_cell_8(const grid_cell::ptr &ref, size_t i) const {return get_neighbour_cell(ref, neighbours_idx_8[i]);}
    grid_cell::ptr get_neighbour_cell_4(const grid_cell::ptr &ref, size_t i) const {return get_neighbour_cell(ref, neighbours_idx_4[i]);}

    void draw();

    inline bool valid_index(const grid_index& idx) const {return idx[0]>=0 && idx[0]<size_[0]
                                                  && idx[1]>=0 && idx[1]<size_[1];}

private:
    grid();
    std::vector<grid_cell::ptr> cells;

    Vector2f center_;
    size2f cell_size_;
    size2i size_;

    grid_index neighbours_idx_4[4];
    grid_index neighbours_idx_8[8];
};

}

#pragma once

#include "grid_cell.h"
#include <vector>

#include "utility.h"

using namespace Eigen;

namespace asn
{

typedef Vector2i size2i;

class grid
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    grid(const Vector2f &center, const size2f &cell_size, const size2i &size);

    ~grid();

    inline size2f get_cell_size() const {return cell_size_;}
    inline Vector2f get_center() const {return center_;}
    inline size2i get_grid_size() const {return size_;}

    inline std::vector<grid_cell::ptr>::iterator begin(){return cells_.begin();}
    inline std::vector<grid_cell::ptr>::iterator end(){return cells_.end();}

    grid_cell::ptr get_cell(const grid_index &idx) const;
    grid_cell::ptr get_neighbour_cell(const grid_cell::ptr &ref, const grid_index &rel_idx) const;

    grid_cell::ptr get_neighbour_cell_8(const grid_cell::ptr &ref, size_t i) const {return get_neighbour_cell(ref, neighbours_idx_8_[i]);}
    grid_cell::ptr get_neighbour_cell_4(const grid_cell::ptr &ref, size_t i) const {return get_neighbour_cell(ref, neighbours_idx_4_[i]);}

    void draw();

    grid_cell::ptr find_cell_contains(const Vector2f &v) const;
    void find_cells_in_rect(const rect &range, std::set<grid_cell::ptr> &cells, bool completely_inside) const;

    inline bool valid_index(const grid_index& idx) const {return idx[0]>=0 && idx[0]<size_[0]
                                                  && idx[1]>=0 && idx[1]<size_[1];}

private:
    grid();
    std::vector<grid_cell::ptr> cells_;

    Vector2f center_;
    size2f cell_size_;
    size2i size_;

    std::vector<grid_index> neighbours_idx_4_;
    std::vector<grid_index> neighbours_idx_8_;
};

}

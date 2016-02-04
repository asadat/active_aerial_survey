#pragma once
#include <memory>
#include <Eigen/Core>

using namespace Eigen;

namespace asn
{

typedef Vector2f size2f;
typedef Vector2i grid_index;

class grid_cell
{
public:
    typedef std::shared_ptr<grid_cell> ptr;

    grid_cell(const Vector2f &center, const size2f &s, const grid_index &index);
    ~grid_cell();

    void draw();

    inline Vector2f get_center() const {return center_;}
    inline size2f get_size() const {return size_;}
    inline grid_index get_index() const {return index_;}

    inline Vector2f get_corner_ul() const { return center_+ Vector2f(-0.5*size_[0], 0.5*size_[1]);}
    inline Vector2f get_corner_ur() const { return center_+ Vector2f( 0.5*size_[0], 0.5*size_[1]);}
    inline Vector2f get_corner_ll() const { return center_+ Vector2f(-0.5*size_[0],-0.5*size_[1]);}
    inline Vector2f get_corner_lr() const { return center_+ Vector2f( 0.5*size_[0],-0.5*size_[1]);}

private:
    grid_cell();

    Vector2f center_;
    size2f size_;
    grid_index index_;
};

}

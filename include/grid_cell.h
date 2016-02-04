#pragma once
#include <memory>
#include <Eigen/Core>

using namespace Eigen;

namespace asn
{

typedef Vector2f size2f;
typedef Vector2i grid_index;

class grid_cell_base
{
public:
    virtual ~grid_cell_base(){}

protected:
    grid_cell_base(){}

    double ground_truth_value_;
    double estimated_value_;
};

class grid_cell: grid_cell_base
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<grid_cell> ptr;

    grid_cell(const Vector2f &center, const size2f &s, const grid_index &index);
    virtual ~grid_cell();

    void draw();

    inline Vector2f get_center() const {return center_;}
    inline size2f get_size() const {return size_;}
    inline grid_index get_index() const {return index_;}

    inline double get_ground_truth_value() const {return ground_truth_value_;}
    inline double get_estimated_value() const {return estimated_value_;}
    inline void set_ground_truth_value(const double &v){ground_truth_value_=v;}
    inline void set_estimated_value(const double &ev){estimated_value_=ev;}

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

#pragma once
#include <memory>
#include <Eigen/Core>
#include "utility.h"

using namespace Eigen;

namespace asn
{

typedef Vector2f size2f;
typedef Vector2i grid_index;

class grid_cell_base
{
public:

    typedef unsigned char label;

    static label generate_label()
    {
        static label l=0;
        return (++l);
    }

public:
    virtual ~grid_cell_base(){}

    inline label get_label() const {return label_;}
    inline void set_label(label l){label_=l;}
    inline bool has_label() const {return (label_>0);}

    inline bool is_visited() const {return visited_;}
    inline void set_visited(bool visited){visited_=visited;}

    inline bool is_covered() const {return covered_;}
    inline void set_covered(bool covered){covered_=covered;}

    inline bool is_ignored() const {return ignored_;}
    inline void set_ignored(bool ignored){ignored_=ignored;}

    inline double get_ground_truth_value() const {return ground_truth_value_;}
    inline double get_estimated_value() const {return estimated_value_;}
    inline void set_ground_truth_value(const double &v){ground_truth_value_=v;}
    inline void set_estimated_value(const double &ev){estimated_value_=ev;}
    inline double get_variance() const {return variance_;}
    inline void set_variance(const double &variance){variance_=variance;}

    int flags_;

protected:
    grid_cell_base():flags_(0), variance_(1), label_(0), visited_(false), covered_(false) {}

    double ground_truth_value_;
    double estimated_value_;
    double variance_;
    label label_;

    /*
     * This means that the cell has been sensed
     * with high resolution
     */
    bool visited_;

    /* This variable indicated that this cell
     * has been covered in the coarse survey
     */
    bool covered_;

    /* This means that the cell should be ignored
     */
    bool ignored_;
};

class grid_cell: public grid_cell_base
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

    inline Vector2f get_corner_ul() const { return center_+ Vector2f(-0.5*size_[0], 0.5*size_[1]);}
    inline Vector2f get_corner_ur() const { return center_+ Vector2f( 0.5*size_[0], 0.5*size_[1]);}
    inline Vector2f get_corner_ll() const { return center_+ Vector2f(-0.5*size_[0],-0.5*size_[1]);}
    inline Vector2f get_corner_lr() const { return center_+ Vector2f( 0.5*size_[0],-0.5*size_[1]);}

    bool is_inside(const rect &r) const;

    inline rect get_rect() const { return rect(center_[0]-0.5*size_[0], center_[1]-0.5*size_[1],
                center_[0]+0.5*size_[0], center_[1]+0.5*size_[1]);}

    bool is_target() const;

private:
    grid_cell();

    Vector2f center_;
    size2f size_;
    grid_index index_;
};

}

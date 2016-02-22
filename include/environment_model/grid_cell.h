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

    inline label get_approx_label() const {return approx_label_;}
    inline void set_approx_label(label l){approx_label_=l;}

    inline bool is_checked_for_skinny() const {return checked_for_skinny_;}
    inline void set_checked_for_skinny(bool checked_for_skinny){checked_for_skinny_=checked_for_skinny;}

    inline bool is_sensed() const {return sensed_;}
    inline void set_sensed(bool sensed){sensed_=sensed;}

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
    inline void set_sensed_time(const double &sensed_time){sensed_time_=(sensed_time<sensed_time_?sensed_time:sensed_time_);}
    inline double get_sensed_time() const {return sensed_time_;}

    int flags_;

protected:
    grid_cell_base():flags_(0), variance_(1), label_(0),
        approx_label_(0), checked_for_skinny_(false), sensed_time_(9e5),
        sensed_(false), covered_(false), ignored_(false) {}

    double ground_truth_value_;
    double estimated_value_;
    double variance_;
    label label_;
    label approx_label_;
    bool checked_for_skinny_;
    double sensed_time_;

    /*
     * This means that the cell has been sensed
     * with high resolution
     */
    bool sensed_;

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

    bool is_uncertain() const;
    bool is_target() const;
    double get_area() const {return size_[0]*size_[1];}

    bool can_be_ignored() const;

private:
    grid_cell();

    Vector2f center_;
    size2f size_;
    grid_index index_;
};

}

#pragma once

#include "utility.h"
#include "environment_model.h"
#include "sensor.h"

namespace asn
{
class mav
{
public:
    mav(environment_model &em, const Vector3f &position);
    virtual ~mav();


    Vector3f get_position() const {return position_;}
    void set_position(const Vector3f &position){position_=position;}

    void sense();

private:

    sensor  sensor_;
    Vector3f position_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

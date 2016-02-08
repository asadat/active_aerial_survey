#pragma once

#include "utility.h"

namespace asn
{
class mav_controller
{
public:
    mav_controller(){}
    virtual ~mav_controller(){}

    Vector3f get_velocity(const Vector3f &goal, const Vector3f &position);

private:

};

}

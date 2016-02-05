
#include "mav.h"

namespace asn
{

mav::mav(environment_model &em, const Vector3f &position):
    sensor_(em),
    position_(position)
{

}

mav::~mav(){}

void mav::sense()
{
    sensor_.sense(position_);
}

}

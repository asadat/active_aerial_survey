#include "mav/mav_controller.h"
#include "active_survey_param.h"

namespace asn
{
Vector3f mav_controller::get_velocity(const Vector3f &goal, const Vector3f &position)
{
    auto dp = goal-position;
    return active_survey_param::speed*dp.normalized();
}

}

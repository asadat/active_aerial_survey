#include "behaviour_controller.h"
#include "mav.h"

namespace asn
{

behaviour_controller::behaviour_controller(mav &m):
    mav_(m)
{
    behaviour_planner_ = behaviour_planner::ptr(new behaviour_planner(mav_));
}

behaviour_controller::~behaviour_controller()
{

}

void behaviour_controller::update(const double &dt)
{
    if(!mav_.at_goal())
    {
        mav_.update_state(dt);
    }
}

}

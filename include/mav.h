#pragma once

#include "utility.h"
#include "environment_model.h"
#include "sensor.h"
#include "mav_controller.h"
#include "behaviour_controller.h"

namespace asn
{

class behaviour_controller;
class behaviour_planner;

class mav
{
public:
    mav(environment_model &em, const Vector3f &position);
    virtual ~mav();


    Vector3f get_position() const {return position_;}
    void set_position(const Vector3f &position){position_=position;}
    Vector3f get_velocity() const {return velocity_;}
    void set_velocity(const Vector3f &velocity){velocity_=velocity;}
    void change_velocity(const Vector3f &d_vel){velocity_+=d_vel;}

    void set_goal(const Vector3f &goal){goal_=goal;}
    Vector3f get_goal() const {return goal_;}

    void sense();
    void update(const double &dt);
    bool at_goal();
    void stop();

    void draw();
private:
    void update_state(const double &dt);

    sensor  sensor_;
    Vector3f position_;
    Vector3f velocity_;
    Vector3f goal_;
    mav_controller mav_controller_;
    behaviour_controller::ptr behaviour_controller_;

    const double goal_reached_threshold_;

    friend class behaviour_controller;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#pragma once

#include <memory>
#include <Eigen/Core>
#include <functional>

using namespace Eigen;

namespace asn
{
class waypoint
{
public:

    enum class action {NONE=0, START_SENSING, STOP_SENSING, INTERRUPT_WAYPOINT};

    typedef std::shared_ptr<waypoint> ptr;
    typedef std::function<void(ptr)> waypoint_call_back;

public:
    explicit waypoint(const Vector3f &pos);
    virtual ~waypoint(){}

    Vector3f operator()() const {return pos_;}

    void set_position(const Vector3f &pos){pos_=pos;}
    Vector3f get_position() const {return pos_;}

    void set_waypoint_call_back(waypoint_call_back cb){waypoint_call_back_=cb;}

    void on_reached_waypoint(ptr waypoint_ptr);

    action get_action() const {return action_;}
    void set_action(const action &a){action_=a;}

    action get_on_set_action() const {return on_set_action_;}
    void set_on_set_action(const action &a){on_set_action_=a;}

protected:
    Vector3f pos_;
    waypoint_call_back waypoint_call_back_;
    action action_;
    action on_set_action_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

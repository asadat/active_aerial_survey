#pragma once

#include <functional>

#include "environment_model/environment_model.h"
#include "environment_model/gaussian_field.h"

namespace asn
{
class sensor
{
public:
    sensor(environment_model & env_model);
    virtual ~sensor();

    void sense(const Vector3f &p, std::function<void(std::set<grid_cell::ptr>&, const Vector3f&)> callback);
    void draw();

private:
    void perform_sense(Vector3f p, std::function<void(std::set<grid_cell::ptr>&, const Vector3f&)> callback);
    rect get_rect(const Vector3f &p) const;

    grid_cell::ptr sense_cell(const Vector2f & p);

    environment_model &environment_model_;
    std::vector<Vector3f> sensing_locations_;

    friend class mav;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

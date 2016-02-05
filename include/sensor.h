#pragma once

#include "environment_model.h"
#include "gaussian_field.h"

namespace asn
{
class sensor
{
public:
    sensor(environment_model & env_model);
    virtual ~sensor();

    void sense(const Vector3f &p);

private:
    environment_model &environment_model_;

    rect get_rect(const Vector3f &p) const;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

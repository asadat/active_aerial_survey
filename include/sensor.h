#pragma once

#include "environment_model.h"

namespace asn
{
class sensor
{
public:
    sensor(environment_model & env_model);
    virtual ~sensor();



private:
    environment_model &environment_model_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

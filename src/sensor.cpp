#include "sensor.h"

namespace asn
{

sensor::sensor(environment_model &env_model):
    environment_model_(env_model)
{

}

sensor::~sensor()
{}

void sensor::sense(const Vector3f &p)
{

}

}

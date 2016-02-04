#pragma once
#include <memory>
#include "gp/gp.h"

namespace asn
{
class gaussian_field: public libgp::GaussianProcess
{
public:
    static std::shared_ptr<gaussian_field> instance()
    {
        if(!instance_)
            instance_ =  std::shared_ptr<gaussian_field>(new gaussian_field());

        return instance_;
    }

    virtual ~gaussian_field();

    void draw();

private:
    static std::shared_ptr<gaussian_field> instance_;
    gaussian_field();
};

}

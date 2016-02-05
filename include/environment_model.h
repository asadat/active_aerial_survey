#pragma once
#include <memory>
#include "grid.h"
#include "active_survey_param.h"

namespace asn
{

class environment_model
{
public:
    static std::shared_ptr<environment_model> instance()
    {
        if(!instance_)
            instance_ = std::shared_ptr<environment_model>(new environment_model());
        return instance_;
    }

    ~environment_model();

    void generate_environment();
    void draw();

private:
    static std::shared_ptr<environment_model> instance_;
    environment_model();

    std::shared_ptr<grid> grid_;

    friend class sensor;
    friend class random_environment_generator;
};

}

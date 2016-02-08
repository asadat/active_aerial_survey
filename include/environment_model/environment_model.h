#pragma once
#include <memory>
#include "environment_model/grid.h"
#include "active_survey_param.h"
#include "math/polygon.h"

#include <mutex>

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

    static void reset(){instance_.reset();}

    ~environment_model();

    void generate_environment();
    void draw();
    void get_environment_polygon(polygon &poly);

private:
    static std::shared_ptr<environment_model> instance_;
    environment_model();

    std::shared_ptr<grid> grid_;
    std::mutex grid_mutex_;

    friend class sensor;
    friend class random_environment_generator;
};

}

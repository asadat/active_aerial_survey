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
    struct dummy_{};

public:
    environment_model(const dummy_ &d);

    static std::shared_ptr<environment_model> instance()
    {
        if(!instance_)
            instance_ = environment_model::make();
        return instance_;
    }

    static void reset(){instance_.reset();}

    ~environment_model();

    void generate_environment(bool randomize=false);
    void draw();
    void get_environment_polygon(polygon &poly);

    static std::shared_ptr<environment_model> make()
    {
        return std::make_shared<environment_model>(dummy_());
    }

private:
    static std::shared_ptr<environment_model> instance_;

    std::shared_ptr<grid> grid_;
    std::mutex grid_mutex_;

    friend class mav;
    friend class sensor;
    friend class random_environment_generator;
};

}

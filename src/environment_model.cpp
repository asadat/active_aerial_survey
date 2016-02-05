#include "environment_model.h"
#include "random_environment_generator.h"
#include "sensor.h"

namespace asn
{

std::shared_ptr<environment_model> environment_model::instance_;

environment_model::environment_model()
{
    size2i s(active_survey_param::area_width/active_survey_param::min_footprint,
             active_survey_param::area_height/active_survey_param::min_footprint);

    grid_ = std::shared_ptr<grid>(new grid({0,0},
            {active_survey_param::min_footprint,active_survey_param::min_footprint}, s));

    generate_environment();
}

environment_model::~environment_model(){}

void environment_model::draw()
{
    if(grid_)
        grid_->draw();
}

void environment_model::generate_environment()
{
    random_environment_generator::generate(*this, active_survey_param::random_seed, 4, active_survey_param::percent_interesting);
}

}


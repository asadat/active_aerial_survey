#include "environment_model.h"
#include "random_environment_generator.h"

namespace asn
{

std::shared_ptr<environment_model> environment_model::instance_;

environment_model::environment_model()
{
    grid_ = std::shared_ptr<grid>(new grid({0,0}, {2,2}, {100,100} ));
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


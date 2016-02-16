#include "environment_model/environment_model.h"
#include "environment_model/random_environment_generator.h"
#include "mav/sensor.h"

namespace asn
{

std::shared_ptr<environment_model> environment_model::instance_;

environment_model::environment_model(const dummy_ &dummy)
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

void environment_model::generate_environment(bool randomize)
{
    double selected_seed=-1;
    if(!randomize)
        selected_seed = active_survey_param::random_seed;

    random_environment_generator::generate(*this, selected_seed, active_survey_param::patches, active_survey_param::percent_interesting);
}

void environment_model::get_environment_polygon(polygon &poly)
{
    auto c = grid_->get_center();

    poly.push_back(c+Vector2f(-0.5*active_survey_param::area_width,
                              -0.5*active_survey_param::area_height));
    poly.push_back(c+Vector2f(+0.5*active_survey_param::area_width,
                              -0.5*active_survey_param::area_height));
    poly.push_back(c+Vector2f(+0.5*active_survey_param::area_width,
                              +0.5*active_survey_param::area_height));
    poly.push_back(c+Vector2f(-0.5*active_survey_param::area_width,
                              +0.5*active_survey_param::area_height));
}

}


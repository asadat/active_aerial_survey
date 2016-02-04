#include "environment_model.h"
namespace asn
{

std::shared_ptr<environment_model> environment_model::instance_;

environment_model::environment_model()
{
    grid_ = std::shared_ptr<grid>(new grid({0,0}, {2,2}, {50,50} ));
}

environment_model::~environment_model(){}

void environment_model::draw()
{
    if(grid_)
        grid_->draw();
}

}


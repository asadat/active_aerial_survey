#include "environment_model/gaussian_field.h"
#include "GL/glut.h"
#include "active_survey_param.h"

using namespace std;

namespace asn
{

shared_ptr<gaussian_field> gaussian_field::instance_;

gaussian_field::gaussian_field():GaussianProcess(2, "CovSum ( CovSEiso, CovNoise)")
{
    Eigen::VectorXd params(covf().get_param_dim());
    params << log(20), log(1), log(active_survey_param::gp_sigma);
    covf().set_loghyper(params);
}

gaussian_field::~gaussian_field()
{

}

void gaussian_field::draw()
{
    return;
    auto s = this->sampleset->size();
    glPointSize(5);
    glColor3f(1,0.5,0);
    glBegin(GL_POINTS);
    for(size_t i=0; i<s; i++)
    {
        auto x = this->sampleset->x(i);
        glVertex3f(x[0], x[1], 1);
    }
    glEnd();
}

}

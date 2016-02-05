#include "grid_cell.h"
#include "utility.h"
#include "active_survey_param.h"

namespace asn
{

grid_cell::grid_cell():grid_cell_base(){}

grid_cell::grid_cell(const Vector2f &center, const size2f &s, const grid_index &index):
    grid_cell_base(),
    center_(center),
    size_(s),
    index_(index)
{
}

grid_cell::~grid_cell(){}

void grid_cell::draw()
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glLineWidth(1.0);
    if(active_survey_param::non_ros::cell_drawing_mode == 0)
        glColor3f(2*ground_truth_value_,2*ground_truth_value_,2*ground_truth_value_);
    else if(active_survey_param::non_ros::cell_drawing_mode == 1)
        glColor3f(2*estimated_value_,2*estimated_value_,2*estimated_value_);
    else if(active_survey_param::non_ros::cell_drawing_mode == 3)
        glColor3f(2*variance_,2*variance_,2*variance_);
    else //if(active_survey_param::non_ros::cell_drawing_mode == 4)
        glColor3f(2*variance_,((1-variance_)<0?0:(1-variance_))*estimated_value_,2*variance_);


    glBegin(GL_QUADS);
    utility::draw_quad(get_rect());
//    utility::gl_vertex2f(get_corner_ll());
//    utility::gl_vertex2f(get_corner_lr());
//    utility::gl_vertex2f(get_corner_ur());
//    utility::gl_vertex2f(get_corner_ul());
    glEnd();


//    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//    glLineWidth(1.0);
//    glColor3f(0.5,0.5,1);
//    glBegin(GL_QUADS);
//    utility::gl_vertex2f(get_corner_ll());
//    utility::gl_vertex2f(get_corner_lr());
//    utility::gl_vertex2f(get_corner_ur());
//    utility::gl_vertex2f(get_corner_ul());
//    glEnd();
}

}

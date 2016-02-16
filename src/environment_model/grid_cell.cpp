#include "environment_model/grid_cell.h"
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
    static const double df_iso =0.05;
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glLineWidth(1.0);
    if(active_survey_param::non_ros::cell_drawing_mode == 0)
        glColor3f(2*ground_truth_value_,
                  2*ground_truth_value_,
                  2*ground_truth_value_);
    else if(active_survey_param::non_ros::cell_drawing_mode == 1)
        glColor3f(2*df_iso*ceil(estimated_value_/df_iso),
                  2*df_iso*ceil(estimated_value_/df_iso),
                  2*df_iso*ceil(estimated_value_/df_iso));
    else if(active_survey_param::non_ros::cell_drawing_mode == 3)
        glColor3f(2*variance_,
                  2*variance_,
                  2*variance_);
    else if(active_survey_param::non_ros::cell_drawing_mode == 4)
        glColor4f(estimated_value_+ variance_, estimated_value_+ variance_,
                  estimated_value_+ variance_,1);
//        glColor4f(2*variance_,
//                  ((1-variance_)<0?0:(1-variance_))*estimated_value_,
//                  2*variance_, ignored_?0.3:1.0);
    else if(active_survey_param::non_ros::cell_drawing_mode == 5)
        utility::gl_color(utility::get_altitude_color(label_));
    else if(active_survey_param::non_ros::cell_drawing_mode == 6)
        utility::gl_color(sensed_&&is_target()?Vector3f(1,1,1):Vector3f(0,0,0));
    else
    {
        if(is_covered())
            glColor3f(0,is_target()?1:0,0);
        else
            glColor3f(0.5,0.5,0.5);
    }

    glBegin(GL_QUADS);
    utility::draw_quad(get_rect());
    glEnd();
}

bool grid_cell::is_target() const
{
    return estimated_value_ > 0.3;
}

bool grid_cell::is_inside(const rect &r) const
{
    return utility::is_rect_inside_rect(get_rect(), r, -0.4*active_survey_param::min_footprint);
}

}

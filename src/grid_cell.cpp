#include "grid_cell.h"
#include "utility.h"

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
    glColor3f(2*ground_truth_value_,2*ground_truth_value_,2*ground_truth_value_);
    glBegin(GL_QUADS);
    utility::gl_vertex2f(get_corner_ll());
    utility::gl_vertex2f(get_corner_lr());
    utility::gl_vertex2f(get_corner_ur());
    utility::gl_vertex2f(get_corner_ul());
    glEnd();
}

}

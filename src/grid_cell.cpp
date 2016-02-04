#include "grid_cell.h"
#include "utility.h"

namespace asn
{

grid_cell::grid_cell(){}

grid_cell::grid_cell(const Vector2f &center, const size2f &s, const grid_index &index):
    center_(center),
    size_(s),
    index_(index)
{

}

grid_cell::~grid_cell(){}

void grid_cell::draw()
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(1.0);
    glColor3f(0,0,0);
    glBegin(GL_QUADS);
    gl_vertex(get_corner_ll());
    gl_vertex(get_corner_lr());
    gl_vertex(get_corner_ur());
    gl_vertex(get_corner_ul());
    glEnd();
}

}

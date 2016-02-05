#pragma once

#include <Eigen/Core>
#include <GL/glut.h>

#include "ros/ros.h"


#define AREA(r) (fabs(r[0]-r[2]) * fabs(r[1]-r[3]))
#define RAND(x,y) (x+((double)(rand()%1000000)*0.000001*(y-x)))

using namespace Eigen;

namespace asn
{

typedef Vector4f rect;

const double epsilon = 0.01;

class utility
{

public:

static bool is_point_inside_rect(const Vector2f &v, const rect &r)
{
    if(v[0]-r[0] > epsilon &&
       v[1]-r[1] > epsilon &&
       r[2]-v[0] > epsilon &&
       r[3]-v[1] > epsilon)
        return true;
    else
        return false;
}

static bool is_rect_inside_rect(const rect &r_in, const rect &r_out)
{
    if(is_point_inside_rect({r_in[0], r_in[1]}, r_out) &&
            is_point_inside_rect({r_in[2], r_in[3]}, r_out))
        return true;
    else
        return false;
}

static void gl_color(const Vector3f &c)
{
    glColor3f(c[0], c[1], c[2]);
}

static void gl_color4f(const Vector4f &c)
{
    glColor4f(c[0], c[1], c[2], c[3]);
}

static void gl_vertex2f(const Vector2f& v)
{
    glVertex2f(v[0], v[1]);
}

static void gl_vertex3f(const Vector3f& v)
{
    glVertex3f(v[0], v[1], v[2]);
}

static void draw_quad(const rect &r)
{
    glVertex2f(r[0],r[1]);
    glVertex2f(r[2],r[1]);
    glVertex2f(r[2],r[3]);
    glVertex2f(r[0],r[3]);
}

static Vector3f get_altitude_color(const double& h)
{
    Vector3f c;
    double small_dh=0.01;
    static std::vector<Vector3f> colors;
    static std::vector<Vector2f> h2c;

    if(colors.empty())
    {
        colors.push_back(Vector3f(0,0,0));
        colors.push_back(Vector3f(0.5,0.5,0.5));
        colors.push_back(Vector3f(0,1,1));
        colors.push_back(Vector3f(1,0,1));
        colors.push_back(Vector3f(0,1,0));
        colors.push_back(Vector3f(0,0,1));
        colors.push_back(Vector3f(1,1,0));
        colors.push_back(Vector3f(1,0,0));

        colors.push_back(Vector3f(160/255.0, 109/255.0, 21/255.0));
        colors.push_back(Vector3f(62/255.0, 102/255.0, 106/255.0));

        std::reverse(colors.begin(), colors.end());
    }

    if(h2c.empty())
    {
        Vector2f i;
        i[0] = h;
        i[1] = 0;
        c = colors[0];
        h2c.push_back(i);
    }
    else
    {
        bool flag = false;
        for(const auto& hc:h2c)
        {
            if(fabs(hc[0]-h) < small_dh)
            {
                c = colors[(int)hc[1]];
                flag = true;
                break;
            }
        }

        if(!flag)
        {
            Vector2f hc;
            hc[0] = h;
            hc[1] = h2c.size();
            h2c.push_back(hc);
            c = colors[h2c.size()];
        }
    }

    return c;
}

};

}

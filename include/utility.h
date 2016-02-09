#pragma once

#include <Eigen/Core>
#include <GL/glut.h>

#include "ros/ros.h"

using namespace Eigen;

namespace asn
{

typedef Vector4f rect;

const double epsilon = 0.01;
const double epsilon_squared = 0.0001;
const int rand_precision = 1000000;
const double rand_precision_inv = 0.000001;


class utility
{

public:

static double point_to_line_distance(Vector2f p1, Vector2f p2, Vector2f x)
{
    double d = fabs((p2[0]-p1[0])*(p1[1]-x[1]) - (p1[0]-x[0])*(p2[1]-p1[1]))/sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]));
    return d;
}

static bool close_enough(const VectorXf &v, const VectorXf &u,
                         const double &d_threshold=epsilon)
{
    return (v-u).squaredNorm() < d_threshold*d_threshold;
}

static double random_number(const double &a, const double &b)
{
    return a+((rand()%rand_precision)*rand_precision_inv*(b-a));
}

static float random_number_f(const float &a, const float &b)
{
    return a+((rand()%rand_precision)*rand_precision_inv*(b-a));
}

static bool is_point_inside_rect(const Vector2f &v, const rect &r, const double &d_thr=epsilon)
{
    if(v[0]-r[0] > d_thr &&
       v[1]-r[1] > d_thr &&
       r[2]-v[0] > d_thr &&
       r[3]-v[1] > d_thr)
        return true;
    else
        return false;
}

static bool is_rect_inside_rect(const rect &r_in, const rect &r_out, const double &d_thr=epsilon)
{
    if(is_point_inside_rect({r_in[0], r_in[1]}, r_out, d_thr) &&
            is_point_inside_rect({r_in[2], r_in[3]}, r_out, d_thr))
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

static void gl_vertex3f(const Vector2f& v, const double & z)
{
    glVertex3f(v[0], v[1], z);
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
            while(colors.size() <= h2c.size())
                colors.push_back(Vector3f(utility::random_number_f(0,1),
                                          utility::random_number_f(0,1),
                                          utility::random_number_f(0,1)));

            c = colors[h2c.size()];
        }
    }

    return c;
}

};

}

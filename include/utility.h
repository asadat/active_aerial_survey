#pragma once

#include <Eigen/Core>
#include <GL/glut.h>

#include "ros/ros.h"

using namespace Eigen;

namespace asn
{

typedef Vector4f rect;

const double epsilon = 1e-2;
const double epsilon_squared = 1e-4;
const int rand_precision = 1e6;
const double rand_precision_inv = 1e-6;


class utility
{

public:

static Vector3f augment(const Vector2f &v, const float &val)
{
    return Vector3f(v[0], v[1], val);
}

static double angle(const Vector2f &a, const Vector2f &b, const Vector2f &c)
{
    auto v = a-b;
    auto u = c-b;
//    auto dot = v.dot(u);
//    auto det = v[0]*u[1]-v[1]*u[0];
//    auto ang = fabs(atan2(det, dot));
//    //ang += ang<0?2*3.1415:0;
//    return ang;

    auto dot1 = (v).dot(v);
    auto dot2 = (u).dot(u);

    auto val = (v).dot(u) / (sqrt(dot1)*sqrt(dot2));
    auto ang = acos(val);
    //if(!(v>=-1 && v<=1))
    if(isnan(ang))
    {
//        ROS_INFO("v: %.2f ang: %.2f dot1: %.2f dot2: %.2f PS: %.2f %.2f %.2f %.2f %.2f %.2f UV: %.2f %.2f %.2f %.2f",
//                 val, ang, dot1, dot2, a[0], a[1], b[0], b[1], c[0], c[1], v[0], v[1], u[0], u[1]);
        return 0;
    }
    return ang;

}

//static double distance_squared(const Vector2f &a, const Vector2f &b)
//{
//    return (a-b).squaredNorm();
//}

template<typename V>
static double distance_squared(const V &a, const V &b)
{
    return (a-b).squaredNorm();
}

template<typename V>
static double distance(const V &a, const V &b)
{
    return (a-b).norm();
}

static bool get_line_segment_intersection(const Vector2f &p0, const Vector2f &p1, const Vector2f &p2,
                                               const Vector2f &p3, Vector2f &intersection_p)
{
    float p0_x = p0[0], p0_y = p0[1], p1_x = p1[0], p1_y=p1[1], p2_x = p2[0], p2_y=p2[1], p3_x = p3[0], p3_y=p3[1];

    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        intersection_p[0] = p0_x + (t * s1_x);
        intersection_p[1] = p0_y + (t * s1_y);
        return true;
    }

    return false;
}

static double point_to_line_distance(const Vector2f &p1, const Vector2f &p2, const Vector2f &x)
{
    //double d = fabs((p2[0]-p1[0])*(p1[1]-x[1]) - (p1[0]-x[0])*(p2[1]-p1[1]))/sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]));
    double d = fabs(point_to_line_signed_distance(p1,p2,x));
    return d;
}

static double point_to_line_signed_distance(const Vector2f &p1, const Vector2f &p2, const Vector2f &x)
{
    double d = ((p2[0]-p1[0])*(p1[1]-x[1]) - (p1[0]-x[0])*(p2[1]-p1[1]))/sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]));
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

static double projected_distance_squared(const Vector2f & v2f, const Vector3f &v3f)
{
    return (v2f[0]-v3f[0])*(v2f[0]-v3f[1])+(v2f[1]-v3f[1])*(v2f[1]-v3f[1]);
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

static void draw_quad(const rect &r, const double &z=0)
{
    glVertex3f(r[0],r[1], z);
    glVertex3f(r[2],r[1], z);
    glVertex3f(r[2],r[3], z);
    glVertex3f(r[0],r[3], z);
}

static void draw_cross(const Vector2f & p, const double &z)
{
    double d = 5;
    Vector2f ll(d,d);
    utility::gl_vertex3f(p-ll, z);
    utility::gl_vertex3f(p+ll, z);

    ll[0] *=-1;
    utility::gl_vertex3f(p-ll, z);
    utility::gl_vertex3f(p+ll, z);
}

static void draw_circle(const Vector2f &c, const double &r, const double &z)
{
    double res=30.0;
    for(int i=0; i<res; i++)
    {
        auto p = c + Vector2f(r*cos(i*3.14*2.0/res), r*sin(i*3.14*2.0/res));
        utility::gl_vertex3f(p,z);
    }
}

static Vector3f get_altitude_color(const double& h)
{
    Vector3f c;
    double small_dh=1e-2;
    static std::vector<Vector3f> colors;
    static std::vector<Vector2f> h2c;

    if(colors.empty())
    {
        //colors.push_back(Vector3f(0,0,0));
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

#ifndef _UTILITY_H
#define _UTILITY_H

#include <Eigen/Core>
#include <GL/glut.h>


#define AREA(r) (fabs(r[0]-r[2]) * fabs(r[1]-r[3]))
#define RAND(x,y) (x+((double)(rand()%1000000)*0.000001*(y-x)))

using namespace Eigen;

namespace asn
{
class utility
{
public:

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

#endif

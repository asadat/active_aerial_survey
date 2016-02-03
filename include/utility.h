#pragma  once

#include <Eigen/Core>

#define AREA(r) (fabs(r[0]-r[2]) * fabs(r[1]-r[3]))

using namespace Eigen;

namespace asn
{

Vector3f get_altitude_color(const double& h)
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

}

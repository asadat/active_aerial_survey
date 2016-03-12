#pragma once

#include <memory>
#include <Eigen/Core>
#include "utility.h"

namespace asn
{

typedef std::vector<Vector2f> polygon;
class polygon_convexhull
{
public:
    static void find_convexhull(const polygon& approximate_polygon, polygon& convexhull, bool &is_line)
    {
        convexhull.clear();
        if(approximate_polygon.size()<3)
        {
            return;
        }

        // first node is the bottom most node
        auto first = std::min_element(approximate_polygon.begin(), approximate_polygon.end(),
                                      [](const Vector2f &v, const Vector2f &u){return v[1]<u[1];});

        convexhull.push_back(*first);


        //second node
        const Vector2f &p2 = convexhull.front();
        const Vector2f &p1 = p2 - Vector2f(-1, 0);

        auto second = std::max_element(approximate_polygon.begin(), approximate_polygon.end(),
                                       [&p1, &p2](const Vector2f &v, const Vector2f &u)
        {
            if(utility::close_enough(p2,u))
                return false;
            else if(utility::close_enough(p2,v))
                return true;

            return utility::angle(p1, p2, v) < utility::angle(p1, p2, u);
        });

        convexhull.push_back(*second);

        //check if the cells form a line a line
        is_line = true;
        const Vector2f &p_0 = approximate_polygon.front();
        const Vector2f &p_1 = approximate_polygon.back();

        for(auto it=approximate_polygon.begin()+2; it!=approximate_polygon.end(); it++)
        {
            auto &p_n = *it;

            if(fabs((p_0[1]-p_1[1])*(p_0[0]-p_n[0]) - (p_0[1]-p_n[1])*(p_0[0]-p_1[0])) > 0.01)
            {
                is_line = false;
                break;
            }
        }

        if(is_line)
        {
            convexhull.clear();
            std::copy(approximate_polygon.begin(), approximate_polygon.end(), std::back_inserter(convexhull));
        }
        else
        {
            auto approximate_polygon_ = approximate_polygon;
            bool added_new_vertex = true;
            // the rest of the nodes
            while(added_new_vertex)
            {
                const auto &p1 = *(--(--convexhull.end()));
                const auto &p2 = convexhull.back();

                const auto next = std::max_element(approximate_polygon_.begin(), approximate_polygon_.end(),
                                                   [&p1, &p2](const Vector2f &v, const Vector2f &u)
                {
                    return utility::angle(p1, p2, v) < utility::angle(p1, p2, u);
                });

                if(next != approximate_polygon_.end())
                {
                }
                else
                    ROS_INFO("CH angle: No min");

                if(next != approximate_polygon_.end())
                {
                    if(utility::close_enough(convexhull.front(), *next))
                        added_new_vertex = false;
                    else
                        convexhull.push_back(*next);
                }
                else
                {
                    added_new_vertex = false;
                }
            }

            if(convexhull.size() == 2)
            {
            }
            else
            {
                // remove extra nodes (colinear edges)
                size_t sz = convexhull.size();

                for(int i=0; i < (int)convexhull.size(); i++)
                {
                    sz = convexhull.size();

                    int i1 = ((i-1)+sz)%sz;
                    int i2 = i;
                    int i3 = (i+1)%sz;

                    Vector2f v = convexhull[i2] - convexhull[i1];
                    Vector2f u = convexhull[i3] - convexhull[i2];

                    Vector3f v3(v[0], v[1], 0.0);
                    Vector3f u3(u[0], u[1], 0.0);

                    auto r = u3.cross(v3);

                    if(sqrt(r.dot(r)) < 0.1)
                    {
                        convexhull.erase(convexhull.begin()+i);
                        i--;
                    }
                }
            }
        }

        std::reverse(convexhull.begin(), convexhull.end());
        auto v_begin = convexhull.back();
        convexhull.pop_back();
        convexhull.insert(convexhull.begin(), v_begin);
    }
};
}

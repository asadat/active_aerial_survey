#pragma once

#include "math/polygon.h"

using namespace Eigen;

namespace asn
{
class trajectory_planner
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::vector<Eigen::Vector3f> trajectory;

    static bool plan_coverage_polygon(const polygon & poly, trajectory &traj)
    {
        return true;
    }

    /*
     * Assumes that the vertices of the rectangle is is in the order of
     * (lower left, lower right, upper right, upper left) and that the
     * sweeping direction is horizontal
     * */
    static bool plan_coverage_rectangle(const polygon & poly, trajectory &traj, const Eigen::Vector2f &footprint, float altitude)
    {
        if(poly.size() != 4) return false;

        double area_height = fabs(poly.front()[1] - poly.back()[1]);
        double area_width = fabs(poly.front()[0] - poly[1][0]);

        int lanes_count = (area_width/footprint[0]);
        ROS_INFO("coverage trajectory planner: #lm_lanes:%d area_width:%f footprint_width:%f", lanes_count, area_width, footprint[0]);

        Vector3f ll(poly.front()[0]+0.5*footprint[0], poly.front()[1]+0.5*footprint[1], altitude);

        for(int i=0; i<lanes_count; i++)
        {
            Vector3f ul = ll + Vector3f(0, area_height-footprint[1],0);

            if(i%2)
            {
                traj.push_back(ul);
                traj.push_back(ll);
            }
            else
            {
                traj.push_back(ll);
                traj.push_back(ul);
            }

            ll += Vector3f(footprint[0],0,0);
        }

        return true;
    }

};

}

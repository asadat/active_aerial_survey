#pragma once

#include "active_survey_param.h"
#include "math/polygon.h"
#include "utility.h"

using namespace Eigen;
using namespace std;

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

    static bool find_base_edge(const polygon &poly, const bool& is_line, size_t &first_index, size_t &second_index, double &convexhull_height)
    {
        if(poly.size() < 3)
            return false;

        if(is_line)
        {
            double max_dist = 0;
            for(size_t i=0; i<poly.size(); ++i)
                for(size_t j=0; j<poly.size(); ++j)
                {
                    double dist = utility::distance_squared(poly[i], poly[j]);
                    if(dist > max_dist)
                    {
                        max_dist = dist;
                        first_index = i;
                        second_index = j;
                    }
                }

            return true;
        }

        size_t size = poly.size();
        double min_height = 1e8;
        size_t min_height_index = 0;

        for(size_t i=0; i < size; i++)
        {
            double max_height = 0;

            for(size_t j=0; j < size; j++)
            {
                if(j==i || (i+1)%size ==j)
                    continue;

                double h = utility::point_to_line_distance(poly[i],
                                                           poly[(i+1)%size],
                        poly[j]);

                max_height = (max_height < h) ? h : max_height;
            }

            if(min_height > max_height)
            {
                min_height = max_height;
                convexhull_height = min_height;
                min_height_index = i;
            }
        }

        double sd = utility::point_to_line_signed_distance(poly[min_height_index],
                                                           poly[(min_height_index+1)%size],
                poly[(min_height_index+2)%size]);
        if(sd > 0)
        {
            first_index = min_height_index;
            second_index = (min_height_index+1)%size;
        }
        else
        {
            second_index = min_height_index;
            first_index = (min_height_index+1)%size;
        }

        return true;
    }

    static bool plan_coverage_path(const polygon &poly, const bool& is_line, const double &inter_lap_distance,
                                   const double &altitude, trajectory &traj, double& coverage_path_cost)
    {
        traj.clear();
        coverage_path_cost=0;

        vector<Vector2f> coverage_path;

        size_t base[2];
        double convexhull_height=0;
        if(!find_base_edge(poly, is_line, base[0], base[1], convexhull_height))
            return false;

        if(is_line)
        {
            traj.push_back(utility::augment(poly[base[0]], altitude));
            traj.push_back(utility::augment(poly[base[1]], altitude));
            coverage_path_cost = utility::distance(traj.front(), traj.back());
            return true;
        }

        Vector2f base_dir  = poly[base[1]]-poly[base[0]];
        Vector2f base_dir_norm = base_dir.normalized();

        //clockwise orthogonal vector
        Vector2f sweep_dir(base_dir_norm[1], -base_dir_norm[0]);

        Vector2f sn0 = poly[base[0]];
        Vector2f en0 = poly[base[1]];
        double offset0 = inter_lap_distance*0.5;

        sn0 += offset0 * sweep_dir;
        en0 += offset0 * sweep_dir;

        // first lm track
        coverage_path.push_back(sn0);
        coverage_path.push_back(en0);

        double n =-1;
        while(n <= ceil((convexhull_height+ 2*active_survey_param::sensing_height) / inter_lap_distance))
        {
            n+=1.0;

            Vector2f sn = sn0;
            Vector2f en = en0;
            sn += n * inter_lap_distance * sweep_dir;
            en += n * inter_lap_distance * sweep_dir;

            sn -=  2 * active_survey_param::area_width * base_dir_norm;
            en +=  2 * active_survey_param::area_width * base_dir_norm;

            vector<Vector2f> intersections;

            for(size_t i=0; i< poly.size(); i++)
            {
                Vector2f ise(0,0);
                if(utility::get_line_segment_intersection(sn, en, poly[i], poly[(i+1)%(poly.size())], ise))
                {
                    intersections.push_back(ise);
                }
            }

            if(intersections.size() > 1)
            {
                if(intersections.size() > 2)
                {
                    for(size_t i=0; i<intersections.size(); i++)
                    {
                        for(size_t j=i+1; j<intersections.size(); j++)
                        {
                            if(utility::distance_squared(intersections[i],intersections[j]) < 0.1)
                            {
                                intersections.erase(intersections.begin()+j);
                                break;
                            }
                        }
                    }
                }

                if(intersections.size()>=2)
                {
                    if(utility::distance_squared(intersections[0],coverage_path[coverage_path.size()-2]) <
                            utility::distance_squared(intersections[1],coverage_path[coverage_path.size()-2]))
                    {
                        coverage_path.push_back(intersections[1]);
                        coverage_path.push_back(intersections[0]);
                    }
                    else
                    {
                        coverage_path.push_back(intersections[0]);
                        coverage_path.push_back(intersections[1]);
                    }
                }
            }
            else if(intersections.size() == 1)
            {

            }
            else
            {
                // no intersection !!!!!!
            }
        }

        if(coverage_path.size() > 2)
        {
            for(auto &p: coverage_path)
            {
                p -= 0.3*(offset0) * sweep_dir;
            }
            coverage_path.erase(coverage_path.begin());
            coverage_path.erase(coverage_path.begin());
        }
        else
        {
            coverage_path[0] -= ((offset0)-convexhull_height/2) * sweep_dir;
            coverage_path[1] -= ((offset0)-convexhull_height/2) * sweep_dir;
        }



        for(auto it=coverage_path.begin(), prev=it; it!=coverage_path.end(); ++it)
        {
            coverage_path_cost += utility::distance(*it, *prev);
            prev = it;
            traj.push_back(utility::augment(*it, altitude));
        }

        coverage_path_cost += (traj.size()-2)* active_survey_param::turning_time;

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

#pragma once
#include <functional>
#include "environment_model.h"
#include "utility.h"

using namespace std;

namespace asn
{

class random_environment_generator
{
public:
    static void generate(environment_model &environment, const long int &seed, const int &cluster_count, const int &percent_interesting)
    {
        auto used_seed = seed;

        if(used_seed<=0)
            used_seed = time(NULL);

        srand(used_seed);


        for(auto it=environment.grid_->begin(); it != environment.grid_->end(); it++)
        {
            (*it)->set_ground_truth_value(0.0);
            (*it)->set_estimated_value(0.0);
        }

        vector<grid_cell::ptr> growing_regions;
        size2i s = environment.grid_->get_grid_size();

        for(int i=0; i<cluster_count; i++)
        {
           int gx = RAND(0, s[0]);
           int gy = RAND(0, s[1]);

           auto c = environment.grid_->get_cell({gx, gy});
           if(c)
           {
               c->set_ground_truth_value(0.5);
               c->set_estimated_value(0.5);
               growing_regions.push_back(c);
           }
        }

        int iterations_count = compute_cellular_automata_steps(s[0]*s[1]*percent_interesting/100.0, cluster_count);
        for(int i=0; i<4*iterations_count; i++)
        {
            cellular_automata_step(environment, growing_regions);
        }


        int d=5;
        for(int i=0; i<s[0]; i++)
        {
            for(int j=0; j<s[1]; j++)
            {
                double p=0;
                int n=0;
                for(int ii=i-d; ii <= i+d; ii++)
                    for(int jj=j-d; jj <= j+d; jj++)
                    {
                        if(ii>=0 && ii<s[0] && jj>=0 && jj<s[1])
                        {
                            n++;
                            p += environment.grid_->get_cell({ii,jj})->get_estimated_value();
                        }
                    }

                environment.grid_->get_cell({i,j})->set_ground_truth_value(p/n);
            }
        }

    }

private:
    static void cellular_automata_step(environment_model &environment, vector<grid_cell::ptr> &regions)
    {
        vector<grid_cell::ptr> newregions;

        for(auto c: regions)
        {
            for(size_t selector=0; selector<4; selector++)
            {
                auto nb = environment.grid_->get_neighbour_cell_4(c, selector);
                if(nb)
                {
                    if(nb->get_ground_truth_value() < 0.4 && nb->get_estimated_value() < 0.4)
                    {
                        if(RAND(0,100) < 50)
                        {
                            nb->set_ground_truth_value(0.5);
                            newregions.push_back(nb);
                        }
                    }
                }
             }
        }

        regions.clear();
        for(auto c: newregions)
        {
            c->set_estimated_value(c->get_ground_truth_value());
            regions.push_back(c);
        }
    }

    static int compute_cellular_automata_steps(const int &fib, const int &clusters_count)
    {
        std::function<int(int)> fidx = [](int sum)->int
        {
            int s=1;
            for(int i=1; ;i++)
            {
                s += i*4;
                if(s > sum)
                    return i;
            }
        };

        return fidx(fib/clusters_count);
    }

private:
    random_environment_generator(){}
};

}

#include "math/graph.h"
#include "environment_model/grid_segment.h"

namespace asn
{

void graph::get_components(const ptr &g, std::vector<ptr> &components)
{
    g->reset_nodes_flags();

    for(auto &n: g->nodes_)
    {
        if(!(n->flags_ & 1))
        {
            ptr c = std::make_shared<graph>();
            c->add_node(n);
            n->flags_ |= 1;
            get_reachable_nodes(n, c);
            components.push_back(c);
        }
    }

}

void graph::get_reachable_nodes(const graph_node::ptr &node, ptr &g)
{
    for(auto it= node->begin_neighbour(); it!=node->end_neighbour(); it++)
    {
        graph_node::ptr n= *it;
        if(!(n->flags_&1))
        {
            n->flags_ |=1;
            g->add_node(n);
            get_reachable_nodes(n, g);
        }
    }
}

void graph::reset_nodes_flags()
{
    for(auto &n: nodes_)
        n->flags_=0;
}

void graph::draw()
{
    if(nodes_.empty())
        ROS_WARN("empty graph!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    auto g_color = utility::get_altitude_color(std::static_pointer_cast<grid_segment>(*nodes_.begin())->get_label());
    for(auto &n: nodes_)
    {
        auto some_cell_center = std::static_pointer_cast<grid_segment>(n)->get_position();

        glColor3f(0.2, 0.2, 1);
        glPointSize(9);
        glBegin(GL_POINTS);
        utility::gl_vertex3f(some_cell_center, 0.2);
        glEnd();

        utility::gl_color(g_color);
        glLineWidth(3);
        glBegin(GL_LINES);
        for(auto i=n->begin_neighbour(); i!=n->end_neighbour(); i++)
        {
            utility::gl_vertex3f(some_cell_center, 0.2);
            utility::gl_vertex3f(std::static_pointer_cast<grid_segment>(*i)->get_position(), 0.2);
        }
        glEnd();
    }
}

}

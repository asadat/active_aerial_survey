#include "math/graph_node.h"

namespace asn
{
graph_node::graph_node():flags_(0)
{}

graph_node::~graph_node()
{}

void graph_node::add_edge(const ptr &v, const ptr &u)
{
    v->neighbours_.insert(u);
    u->neighbours_.insert(v);
}

}

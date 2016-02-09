#pragma once

#include "math/graph_node.h"

namespace asn
{

class graph
{
public:
    typedef std::shared_ptr<graph> ptr;
    typedef std::set<graph_node::ptr>::const_iterator const_node_iterator;
    typedef std::set<graph_node::ptr>::iterator node_iterator;

    graph(){}
    ~graph(){}

    const_node_iterator begin() const {return nodes_.begin();}
    const_node_iterator end() const {return nodes_.end();}

    void add_node(const graph_node::ptr &node){nodes_.insert(node);}

    static void get_components(const ptr &g, std::vector<ptr> &components);
    static void get_reachable_nodes(const graph_node::ptr & node, ptr &g);

    void draw();

private:
    void reset_nodes_flags();
    std::set<graph_node::ptr> nodes_;
};

}

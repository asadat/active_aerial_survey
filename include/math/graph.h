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

    node_iterator begin() const {return nodes_.begin();}
    node_iterator end() const {return nodes_.end();}

    void add_node(const graph_node::ptr &node){nodes_.insert(node);}
    void merge_nodes(graph_node::ptr u, graph_node::ptr merging_node);
    static void get_components(const ptr &g, std::vector<ptr> &components);
    static void get_reachable_nodes(const graph_node::ptr & node, ptr &g);

    void draw();
    size_t size() const {return nodes_.size();}

    /*
     * Remove node from the nodes list
     * NOTE: this should be used only
     * when the edges are already removed
     * from the graph
     * */
    void remove_left_alone_node(graph_node::ptr node){nodes_.erase(node);}

private:
    void reset_nodes_flags();
    std::set<graph_node::ptr> nodes_;
};

}

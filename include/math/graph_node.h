#pragma once

#include <Eigen/Core>
#include <memory>
#include <set>

namespace asn
{

class graph_node
{
public:
    typedef std::shared_ptr<graph_node> ptr;
    typedef std::set<ptr>::const_iterator const_neighbour_iterator;
    typedef std::set<ptr>::iterator neighbour_iterator;

    graph_node();
    virtual ~graph_node();

    inline neighbour_iterator begin_neighbour(){return neighbours_.begin();}
    inline neighbour_iterator end_neighbour(){return neighbours_.end();}

    static void add_edge(const ptr &v, const ptr &u);

    virtual Eigen::Vector2f get_position() const {return {0,0};}

protected:
    std::set<graph_node::ptr> neighbours_;

private:
    int flags_;

    friend class graph;
};

}

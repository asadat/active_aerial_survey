#pragma once

#include "environment_model/grid.h"
#include "math/graph.h"
//#include "math/graph_node.h"
#include "math/polygon.h"

#include <set>
#include <functional>


namespace asn
{

//class graph;

class grid_segment: public graph_node
{
public:
    typedef std::shared_ptr<grid_segment> ptr;
    typedef std::set<grid_cell::ptr>::const_iterator const_cell_iterator;
    typedef std::set<grid_cell::ptr>::iterator cell_iterator;
    typedef polygon::const_iterator const_polygon_vertex_iterator;

public:
    grid_segment(grid &grd, grid_cell::ptr seed_cell, grid_cell_base::label label);
    virtual ~grid_segment();

    void grow(std::function<grid_segment::ptr(grid_cell_base::label)> segments_accessor);

    void find_approximate_polygon();
    void find_convexhull();
    bool plan_coverage_path(const double &inter_lap_distance, const double &altitude);

    void get_approximate_polygon(polygon &approx_poly) const;
    void get_convexhull(polygon &convexhull) const;

    inline const_cell_iterator begin() const {return cells_.begin();}
    inline const_cell_iterator end() const {return cells_.end();}

    inline const_polygon_vertex_iterator begin_approx_poly() const {return approximate_polygon_.begin();}
    inline const_polygon_vertex_iterator end_approx_poly() const {return approximate_polygon_.end();}

    inline const_polygon_vertex_iterator begin_convexhull() const {return convexhull_.begin();}
    inline const_polygon_vertex_iterator end_convexhull() const {return convexhull_.end();}

    static inline bool is_visited(grid_cell::ptr &cell){return cell->flags_&1;}
    static inline void set_visited(grid_cell::ptr &cell){cell->flags_ |= 1;}
    static inline void reset_visited(grid_cell::ptr &cell){cell->flags_ &= 0;}

    virtual Vector2f get_position(){return (*begin())->get_center();}

    grid_cell_base::label get_label() const {return (*begin())->get_label();}

    static void merge_with_segment(ptr u, ptr merging_segment, graph::ptr &component);

    void clear();
    static void reset_cell(grid_cell::ptr cell);

    size_t get_cell_count() const {return cells_.size();}
    size_t get_convexhull_vertices_count() const {return  convexhull_.size();}
    size_t get_approx_poly_vertices_count() const {return  approximate_polygon_.size();}

    bool is_valid() const {return get_convexhull_vertices_count() > 3;}

    void draw();

    double get_coverage_path_cost(const Vector3f &from) const;
    double get_coverage_path_switching_cost(const Vector3f &from) const;
    double get_reaching_cost(const Vector3f &from) const;
    double get_segment_value() const;
    void get_coverage_path(std::vector<Vector3f> &coverage_path) const;
    inline size_t get_coverage_path_waypoint_count() const {return coverage_path_.size();}

    void set_ignored();
    inline bool get_ignored() const {return (*begin())->is_ignored();}

    inline void set_selected(bool selected){is_selected_=selected;}
    inline bool is_selected() const {return is_selected_;}

    inline Vector3f get_color() const {return utility::get_altitude_color(get_label());}

    template<class OutIterator>
    void get_uncertain_neighbour_cells(OutIterator out_iterator);

    double get_uncertain_neighbour_area() const;

private:
    grid_segment()=delete;
    void remove_skinny_part(grid_cell::ptr cell);
    int get_approximate_neighbours_count(grid_cell::ptr cell);
    void set_label(grid_cell_base::label l);
    bool find_base_edge(size_t &first_index, size_t &second_index, double &convexhull_height) const;

    //template<class OutIterator>
    //void find_uncertain_neighbour_cells(OutIterator out_iterator);

    grid &grid_;
    std::set<grid_cell::ptr> cells_;
    std::vector<grid_cell::ptr> boundary_cells_;
    std::vector<grid_cell::ptr> uncertain_boundary_;
    std::vector<grid_cell::ptr> approximate_poly_cells_;

    double coverage_path_cost_;
    std::vector<Vector3f> coverage_path_;
    bool is_line_;

    bool is_selected_;

    polygon approximate_polygon_;
    polygon convexhull_;


};

}

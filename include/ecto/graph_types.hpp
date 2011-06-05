#pragma once

#include <deque>

//this quiets a deprecated warning
#define BOOST_NO_HASH
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/foreach.hpp>
#include <boost/unordered_map.hpp>

#include <ecto/module.hpp>

namespace ecto {
  namespace graph {

    struct edge
    {
      edge(const std::string& fp, const std::string& tp) :
        from_port(fp), to_port(tp)
      { }

      std::string from_port, to_port;
      std::deque<ecto::tendril> deque;
      typedef boost::shared_ptr<edge> ptr;
      typedef boost::shared_ptr<const edge> const_ptr;
    };


    // if the first argument is a sequence type (vecS, etc) then parallel edges are allowed
    typedef boost::adjacency_list<boost::vecS, // OutEdgeList...
                                  boost::vecS, // VertexList
                                  boost::bidirectionalS, // Directed
                                  module::ptr, // vertex property
                                  edge::ptr> // edge property
    graph_t;
  



    struct vertex_writer
    {
      graph_t* g;

      vertex_writer(graph_t* g_) : g(g_) { }

      void operator()(std::ostream& out, graph_t::vertex_descriptor vd)
      {
        out << "[label=\"" << (*g)[vd]->name() << "\"]";
      }
    };

    struct edge_writer
    {
      graph_t* g;

      edge_writer(graph_t* g_) :
        g(g_)
      { }

      void operator()(std::ostream& out, graph_t::edge_descriptor ed)
      {
        out << "[headlabel=\"" << (*g)[ed]->to_port << "\" taillabel=\"" << (*g)[ed]->from_port << "\"]";
      }
    };

    struct graph_writer
    {
      void operator()(std::ostream& out) const
      {
        out << "graph [rankdir=TB, ranksep=1]" << std::endl;
        out << "edge [labelfontsize=8]" << std::endl;
      }
    };

  }
}

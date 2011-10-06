#pragma once

#include <ecto/plasm.hpp>
#include <boost/tr1/unordered_map.hpp>

namespace ecto {

  struct plasm::impl
  {
    impl();

    //insert a cell into the graph, will retrieve the
    //vertex descriptor if its already in the graph...
    graph::graph_t::vertex_descriptor insert_module(cell::ptr m);

    void connect(cell::ptr from, std::string output, cell::ptr to, std::string input);

    void disconnect(cell::ptr from, std::string output, cell::ptr to, std::string input);

    //the cell to vertex mapping
    //unordered_map so that cell ptr works as a key...
    typedef boost::unordered_map<ecto::cell::ptr, graph::graph_t::vertex_descriptor> ModuleVertexMap;
    ModuleVertexMap mv_map;
    graph::graph_t graph;
    boost::shared_ptr<ecto::schedulers::singlethreaded> scheduler;

  };
}

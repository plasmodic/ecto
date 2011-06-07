#pragma once

#include <ecto/graph_types.hpp>

namespace ecto {
  namespace scheduler {

    int 
    invoke_process(graph::graph_t& graph, graph::graph_t::vertex_descriptor vd);

  }
}

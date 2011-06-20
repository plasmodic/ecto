#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/module.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

#include <ecto/graph_types.hpp>
#include <ecto/plasm.hpp>

namespace ecto {

  using namespace ecto::graph;

  namespace scheduler {

    int 
    invoke_process(graph_t& graph, graph_t::vertex_descriptor vd)
    {
      module::ptr m = graph[vd];

      graph_t::in_edge_iterator inbegin, inend;
      tie(inbegin, inend) = boost::in_edges(vd, graph);
      while (inbegin != inend)
        {
          edge::ptr e = graph[*inbegin];
          m->inputs.at(e->to_port)->copy_value(e->front());
          e->pop_front();
          ++inbegin;
        }

      int val = m->process();

      graph_t::out_edge_iterator outbegin, outend;
      tie(outbegin, outend) = boost::out_edges(vd, graph);
      while (outbegin != outend)
        {
          edge::ptr e = graph[*outbegin];
          e->push_back(*m->outputs.at(e->from_port));
          ++outbegin;
        }

      return val;
    }

  }
}

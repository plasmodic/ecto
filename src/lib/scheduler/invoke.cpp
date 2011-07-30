#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/cell.hpp>

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
      cell::ptr m = graph[vd];

      graph_t::in_edge_iterator inbegin, inend;
      tie(inbegin, inend) = boost::in_edges(vd, graph);
      while (inbegin != inend)
        {
          edge::ptr e = graph[*inbegin];
          tendril& from = e->front();
          tendril& to = *(m->inputs[e->to_port]);
          to << from;
          e->pop_front();
          ++inbegin;
        }
      //verify that all inputs have been set.
      m->verify_inputs();

      int val = m->process();

      graph_t::out_edge_iterator outbegin, outend;
      tie(outbegin, outend) = boost::out_edges(vd, graph);
      while (outbegin != outend)
        {
          edge::ptr e = graph[*outbegin];
          e->push_back(*m->outputs[e->from_port]);//copy everything... value, docs, user_defined, etc...
          ++outbegin;
        }

      return val;
    }

  }
}

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
#include "plasm_impl.hpp"
#include <ecto/scheduler/singlethreaded.hpp>


namespace ecto {

  using namespace ecto::graph;

  namespace scheduler {

    singlethreaded::singlethreaded(plasm& p) 
      : graph(p.impl_->graph) 
    { }
    singlethreaded:: singlethreaded(graph::graph_t& g_) : graph(g_) { }

    int 
    singlethreaded::invoke_process(graph_t::vertex_descriptor vd)
    {
      module::ptr m = graph[vd];

      graph_t::in_edge_iterator inbegin, inend;
      tie(inbegin, inend) = boost::in_edges(vd, graph);
      while (inbegin != inend)
        {
          edge::ptr e = graph[*inbegin];
          m->inputs.at(e->to_port).copy_value(e->deque.front());
          e->deque.pop_front();
          ++inbegin;
        }
      int val = m->process();

      graph_t::out_edge_iterator outbegin, outend;
      tie(outbegin, outend) = boost::out_edges(vd, graph);
      while (outbegin != outend)
        {
          edge::ptr e = graph[*outbegin];
          e->deque.push_back(m->outputs.at(e->from_port));
          ++outbegin;
        }

      return val;
    }

    void singlethreaded::compute_stack()
    {
      if (!stack.empty()) //will be empty if this needs to be computed.
        return;
      boost::topological_sort(graph, std::back_inserter(stack));
      std::reverse(stack.begin(), stack.end());
    }

    int singlethreaded::execute()
    {
      //compute ordering
      compute_stack();
      for (size_t k = 0; k < stack.size(); ++k)
        {
          //need to check the return val of a process here, non zero means exit...
          size_t retval = invoke_process(stack[k]);
          if (retval)
            return retval;
        }
      return 0;
    }
  }
}

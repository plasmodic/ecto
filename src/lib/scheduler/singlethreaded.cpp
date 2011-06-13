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
#include <ecto/scheduler/invoke.hpp>
#include <ecto/scheduler/singlethreaded.hpp>


namespace ecto {

  using namespace ecto::graph;

  namespace scheduler {

    singlethreaded::singlethreaded(plasm& p) 
      : graph(p.graph()) 
    { }

    int 
    singlethreaded::invoke_process(graph_t::vertex_descriptor vd)
    {
      return ecto::scheduler::invoke_process(graph, vd);
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
      while(true) {
        for (size_t k = 0; k < stack.size(); ++k)
          {
            //need to check the return val of a process here, non zero means exit...
            size_t retval = invoke_process(stack[k]);
            if (retval)
              return retval;
          }
      }
      return 0;
    }

    int singlethreaded::execute(unsigned j)
    {
      compute_stack();

      for (unsigned r=0; r<j; ++r)
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

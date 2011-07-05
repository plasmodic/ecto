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
#include <ecto/scheduler/invoke.hpp>
#include <ecto/scheduler/singlethreaded.hpp>


namespace ecto {

  using namespace ecto::graph;

  namespace scheduler {

    singlethreaded::singlethreaded(plasm& p) 
      : plasm_(p), graph(p.graph()) 
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
      //check this plasm for correctness.
      plasm_.check();
      boost::topological_sort(graph, std::back_inserter(stack));
      std::reverse(stack.begin(), stack.end());
    }

    static bool interupt;
    static void sigint_static_thunk(int)
    {
      std::cerr << "*** SIGINT received, stopping graph execution.\n"
                << "*** If you are stuck here, you may need to hit ^C again\n"
                << "*** when back in the interpreter thread.\n"
                << "*** or Ctrl-\\ (backslash) for a hard stop.\n"
                << std::endl;
      interupt = true;
    }

    int singlethreaded::execute()
    {
      interupt = false;
      signal(SIGINT, &sigint_static_thunk);
      //compute ordering
      compute_stack();
      while(true) {
        for (size_t k = 0; k < stack.size(); ++k)
          {
            //need to check the return val of a process here, non zero means exit...
            size_t retval = invoke_process(stack[k]);
            if (retval)
              return retval;
            if(interupt) return interupt;
          }
      }
      return 0;
    }

    int singlethreaded::execute(unsigned j)
    {
      interupt = false;
      signal(SIGINT, &sigint_static_thunk);

      compute_stack();
      for (unsigned r=0; r<j; ++r)
        for (size_t k = 0; k < stack.size(); ++k)
          {
            //need to check the return val of a process here, non zero means exit...
            size_t retval = invoke_process(stack[k]);
            if (retval)
              return retval;
            if(interupt) return interupt;
          }
      return 0;
    }
  }
}

#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/module.hpp>
#include <ecto/graph_types.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>



namespace ecto {

  namespace scheduler {
    
    struct singlethreaded 
    {
      plasm& plasm_;
      ecto::graph::graph_t& graph;
      
      singlethreaded(plasm&);

      int invoke_process(ecto::graph::graph_t::vertex_descriptor vd);
      void compute_stack();
      int execute();
      int execute(unsigned niter);
      std::vector<ecto::graph::graph_t::vertex_descriptor> stack;
    };
  }
}

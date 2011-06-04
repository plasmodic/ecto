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
      ecto::graph::graph_t& graph;
      
      singlethreaded(ecto::graph::graph_t&);

      int invoke_process(ecto::graph::graph_t::vertex_descriptor vd);
      void compute_stack();
      int execute();
      std::vector<ecto::graph::graph_t::vertex_descriptor> stack;
    };
  }
}

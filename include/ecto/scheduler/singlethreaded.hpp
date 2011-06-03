#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/module.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

#include <ecto/graph_types.hpp>


namespace ecto {

  namespace scheduler {
    
    struct singlethreaded 
    {
      ecto::graph::graph_t& graph;
      
      singlethreaded(graph::graph_t&);

      int invoke_process(graph_t::vertex_descriptor vd);
      void compute_stack();
      int execute();
      std::vector<graph_t::vertex_descriptor> stack;
    };
  }
}

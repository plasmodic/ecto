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
#include <ecto/scheduler/threadpool.hpp>


namespace ecto {

  using namespace ecto::graph;

  namespace scheduler {

    struct process_invoker 
    {
      graph_t& g;
      graph_t::vertex_descriptor vd;
      module::ptr m;

      process_invoker(graph_t& g_, graph_t::vertex_descriptor vd_)
        : g(g_), vd(vd_)
      { 
        m = g[vd];
      } 

      void operator()() {
        std::cout << __PRETTY_FUNCTION__ << "\n";
      }
    };

    struct threadpool::impl 
    {
      
    };

    threadpool::threadpool(plasm& p) 
      : graph(p.graph()), impl_(new impl)
    { }

    int threadpool::execute()
    {
      graph_t::vertex_iterator begin, end;
      for (tie(begin, end) = vertices(graph);
           begin != end;
           ++begin)
        {
          
        }

      return 0;
    }
  }
}

#include <boost/make_shared.hpp>
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

    struct threadpool::impl 
    {
      struct invoker 
      {
        typedef boost::shared_ptr<invoker> ptr;

        graph_t& g;
        graph_t::vertex_descriptor vd;
        module::ptr m;

        invoker(graph_t& g_, graph_t::vertex_descriptor vd_)
          : g(g_), vd(vd_)
        { 
          m = g[vd];
          std::cout << "invoker, m=" << m->name() << " @ " << m.get() << "\n";
        } 

        void operator()() {
          std::cout << __PRETTY_FUNCTION__ << "\n";
        }
      };

      std::map<graph_t::vertex_descriptor, invoker::ptr> invokers;
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
          std::cout << "vertex: " << *begin << "\n";
          impl::invoker::ptr ip(new impl::invoker(graph, *begin));
          impl_->invokers[*begin] = ip;
        }

      return 0;
    }
  }
}

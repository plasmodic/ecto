#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/module.hpp>

#include <ecto/graph_types.hpp>
#include <ecto/plasm.hpp>
#include <ecto/scheduler/invoke.hpp>
#include <ecto/scheduler/threadpool.hpp>

#include <boost/spirit/home/phoenix/core.hpp>
#include <boost/spirit/home/phoenix/operator.hpp>

namespace ecto {

  using namespace ecto::graph;

  namespace scheduler {

    struct threadpool::impl 
    {
      typedef boost::function<bool(unsigned)> respawn_cb_t;

      struct invoker : boost::noncopyable
      {
        typedef boost::shared_ptr<invoker> ptr;

        boost::asio::io_service& serv;
        graph_t& g;
        graph_t::vertex_descriptor vd;
        unsigned n_calls;
        respawn_cb_t respawn;
        boost::mutex mtx;

        invoker(boost::asio::io_service& serv_, graph_t& g_, graph_t::vertex_descriptor vd_,
                respawn_cb_t respawn_)
          : serv(serv_), g(g_), vd(vd_), n_calls(0), respawn(respawn_)
        { }

        void async_wait_for_input()
        {
          boost::mutex::scoped_lock lock(mtx);
          namespace asio = boost::asio;

          // keep outer run() from returning
          asio::io_service::work work(serv);

          asio::deadline_timer dt(serv);
          if (inputs_ready()) {
            serv.post(boost::bind(&invoker::invoke, this));
          } else {
            dt.expires_from_now(boost::posix_time::milliseconds(500));
            dt.async_wait(boost::bind(&invoker::async_wait_for_input, this));
          }
        }

        void invoke()
        {
          boost::mutex::scoped_lock lock(mtx);
          ecto::scheduler::invoke_process(g, vd);
          ++n_calls;
          if (respawn(n_calls)) 
            {
              serv.post(boost::bind(&invoker::async_wait_for_input, this));
            }
        }
        
        bool inputs_ready() 
        {
          graph_t::in_edge_iterator in_beg, in_end;
          for (tie(in_beg, in_end) = in_edges(vd, g);
               in_beg != in_end; ++in_beg)
            {
              //std::cout << "in_beg:" << *in_beg << "\n";
              graph::edge::ptr e = g[*in_beg];
              //std::cout << "in:" << e->from_port << " >> " << e->to_port << " (" << e->size() << ")\n";
              if (e->size() == 0)
                return false;
            }

          graph_t::out_edge_iterator out_beg, out_end;
          for (tie(out_beg, out_end) = out_edges(vd, g);
               out_beg != out_end; ++out_beg)
            {
              // std::cout << "in_beg:" << *in_beg << "\n";
              graph::edge::ptr e = g[*out_beg];
              //std::cout << "out: " << e->from_port << " >> " << e->to_port << " (" << e->size() << ")\n";
              if (e->size() > 0)
                return false;
            }

          // std::cout << "inputs ready!\n";
          return true;
        }

        ~invoker() { }

      }; // struct invoker

      static void runservice(boost::asio::io_service& s, unsigned n) 
      { 
        // std::cout << ">>> run service " << n << std::endl;
        s.run();
        // std::cout << "<<< run service " << n << std::endl;
      }
      
      int execute(unsigned nthreads, impl::respawn_cb_t respawn, graph_t& graph)
      {
        namespace asio = boost::asio;

        graph_t::vertex_iterator begin, end;
        for (tie(begin, end) = vertices(graph);
             begin != end;
             ++begin)
          {
            std::cout << "vertex: " << *begin << "\n";
            impl::invoker::ptr ip(new impl::invoker(serv, graph, *begin, respawn));
            invokers[*begin] = ip;
            ip->async_wait_for_input();
          }

        boost::thread_group tgroup;

        { 
          asio::io_service::work work(serv);

          for (unsigned j=0; j<nthreads; ++j)
            {
              // std::cout << "starting thread...\n";
              tgroup.create_thread(boost::bind(&runservice, boost::ref(serv), j));
            }

        } // let work go out of scope...   invokers now have their own work on serv

        tgroup.join_all();

        return 0;
      }

      ~impl() 
      {
        // be sure your invokers disappear before you do (you're holding the main service)
        invokers_t().swap(invokers);
      }

      typedef std::map<graph_t::vertex_descriptor, invoker::ptr> invokers_t;
      invokers_t invokers;
      boost::asio::io_service serv;
    };

    threadpool::threadpool(plasm& p)
      : graph(p.graph()), impl_(new impl)
    { }

    namespace phx = boost::phoenix;

    int threadpool::execute(unsigned nthreads)
    {
      return impl_->execute(nthreads, phx::val(true), graph);
    }

    int threadpool::execute(unsigned nthreads, unsigned ncalls)
    {
      return impl_->execute(nthreads, boost::phoenix::arg_names::arg1 < ncalls, graph);
    }

  }
}

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


namespace ecto {

  using namespace ecto::graph;

  namespace scheduler {

    struct threadpool::impl 
    {
      struct invoker : boost::noncopyable
      {
        typedef boost::shared_ptr<invoker> ptr;

        boost::asio::io_service& serv;
        boost::asio::io_service::work work;
        graph_t& g;
        graph_t::vertex_descriptor vd;
        module::ptr m;
        boost::thread watcher_thread;

        invoker(boost::asio::io_service& serv_, graph_t& g_, graph_t::vertex_descriptor vd_)
          : serv(serv_), work(serv), g(g_), vd(vd_)
        { 
          m = g[vd];
          std::cout << "invoker, m=" << m->name() << " @ " << m.get() << "\n";
        } 

        void start()
        {
          watcher_thread.join();
          boost::thread new_watcher(boost::bind(&invoker::async_wait_for_input, this));
          watcher_thread.swap(new_watcher);
        }

        void async_wait_for_input() 
        {
          namespace asio = boost::asio;
          asio::io_service tserv;
          asio::deadline_timer dt(tserv);
          asio::io_service::work work(tserv);
          for (;;) {
            if (inputs_ready()) {
              serv.post(boost::bind(&invoker::invoke, this));
              return;
            }
            dt.expires_from_now(boost::posix_time::milliseconds(1000));
            dt.wait();
            std::cout << "nah" << std::endl;
          }
        }

        void invoke()
        {
          std::cout << __PRETTY_FUNCTION__ << "\n";
          ecto::scheduler::invoke_process(g, vd);
          start();
        }
        
        bool inputs_ready() 
        {
          graph_t::in_edge_iterator in_beg, in_end;
          for (tie(in_beg, in_end) = in_edges(vd, g);
               in_beg != in_end; ++in_beg)
            {
              std::cout << "in_beg:" << *in_beg << "\n";
              graph::edge::ptr e = g[*in_beg];
              std::cout << e->from_port << " >> " << e->to_port << " (" << e->deque.size() << ")\n";
              if (e->deque.size() == 0)
                return false;
            }
          std::cout << "inputs ready!\n";
          return true;
        }
      };

      std::map<graph_t::vertex_descriptor, invoker::ptr> invokers;
      boost::asio::io_service serv;
    };

    threadpool::threadpool(plasm& p) 
      : graph(p.graph()), impl_(new impl)
    { }

    int threadpool::execute(unsigned nthreads)
    {
      namespace asio = boost::asio;

      boost::thread_group tgroup;

      for (unsigned j=0; j<nthreads; ++j)
        {
          std::cout << "starting thread...\n";
          tgroup.create_thread(boost::bind(&boost::asio::io_service::run, boost::ref(impl_->serv)));
        }


      graph_t::vertex_iterator begin, end;
      for (tie(begin, end) = vertices(graph);
           begin != end;
           ++begin)
        {
          std::cout << "vertex: " << *begin << "\n";
          impl::invoker::ptr ip(new impl::invoker(impl_->serv, graph, *begin));
          ip->start();
          impl_->invokers[*begin] = ip;
        }

      tgroup.join_all();

      return 0;
    }
  }
}

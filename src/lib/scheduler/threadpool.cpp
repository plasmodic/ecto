#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>

#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/module.hpp>
#include <ecto/log.hpp>
#include <ecto/strand.hpp>

#include <ecto/graph_types.hpp>
#include <ecto/plasm.hpp>
#include <ecto/scheduler/invoke.hpp>
#include <ecto/scheduler/threadpool.hpp>

#include <boost/spirit/home/phoenix/core.hpp>
#include <boost/spirit/home/phoenix/operator.hpp>
#include <boost/exception/all.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace ecto {

  using namespace ecto::graph;
  using boost::bind;
  using boost::ref;
  using boost::shared_ptr;
  using boost::thread;
  using boost::exception;
  using boost::exception_ptr;

  namespace scheduler {
    namespace asio = boost::asio;

    namespace pt = boost::posix_time;

    struct threadpool::impl 
    {
      typedef boost::function<bool(unsigned)> respawn_cb_t;


      struct propagator
      {
        asio::io_service &from, &to;
        asio::io_service::work work;

        propagator(asio::io_service& from_, asio::io_service& to_) 
          : from(from_), to(to_), work(to) { }

        void operator()() {
          from.run();
        }

        template <typename Handler>
        void post(Handler h)
        {
          to.post(h);
        }
      };

      struct thrower
      {
        exception_ptr eptr;
        thrower(exception_ptr eptr_) : eptr(eptr_) { }

        void operator()() const
        {
          rethrow_exception(eptr);
        }
      };

      struct stopper 
      {
        typedef void result_type;

        void operator()(asio::io_service& serv) const
        {
          serv.stop();
        }
      };


      struct runandjoin
      {
        typedef shared_ptr<runandjoin> ptr;

        thread runner;

        runandjoin() 
        { 
          runner.join();
        }

        ~runandjoin() {
          runner.join();
        }

        void joinit() 
        {
          runner.join();
        }

        template <typename Work>
        void impl(Work w)
        {
          //          std::cout << __PRETTY_FUNCTION__ << "\n";
          try {
            w();
          } catch (const boost::exception& e) {
            w.post(thrower(boost::current_exception()));
          } catch (const std::exception& e) {
            w.post(thrower(boost::current_exception()));
          }
          w.post(bind(&runandjoin::joinit, this));
        }

        template <typename Work>
        void run(Work w)
        {
          thread* newthread = new thread(bind(&runandjoin::impl<Work>, this, w));
          newthread->swap(runner);
        }
      };

      struct invoker : boost::noncopyable
      {
        typedef boost::shared_ptr<invoker> ptr;

        threadpool::impl& context;

        // boost::asio::deadline_timer dt;
        graph_t& g;
        graph_t::vertex_descriptor vd;
        unsigned n_calls;
        respawn_cb_t respawn;

        invoker(threadpool::impl& context_,
                graph_t& g_, 
                graph_t::vertex_descriptor vd_,
                respawn_cb_t respawn_)
          : context(context_), /*dt(context.workserv),*/ g(g_), vd(vd_), n_calls(0), respawn(respawn_)
        { }

        template <typename Handler>
        void post(Handler h)
        {
          module::ptr m = g[vd];
          if (m->strand_)
            {
              const ecto::strand& skey = *(m->strand_);
              boost::shared_ptr<asio::io_service::strand>& strand_p = context.strands[skey];
              if (!strand_p) {
                strand_p.reset(new boost::asio::io_service::strand(context.workserv));
              }
              strand_p->post(h);
            }
          else
            context.workserv.post(h);
        }

        void async_wait_for_input()
        {
          ECTO_LOG_DEBUG("%s async_wait_for_input", this);
          namespace asio = boost::asio;

          if (context.stop) {
            post(bind(&invoker::destroy, this));
            return;
          }
            
          if (inputs_ready()) {
            ECTO_LOG_DEBUG("%s inputs ready", this);
            post(bind(&invoker::invoke, this));
          } else {
            context.workserv.post(bind(&invoker::async_wait_for_input, this));
          }
        }

        void destroy()
        {
          module::ptr m = g[vd];
          // FIXME: not catching exceptions possibly thrown by destroy
          m->destroy();
        }

        void invoke()
        {
          ECTO_LOG_DEBUG("%s invoke", this);
          try {
            int j = ecto::scheduler::invoke_process(g, vd);
            if (j != ecto::OK)
              {
                std::cout << "Module " << g[vd]->name() << " returned not okay. Stopping everything." 
                          << std::endl; 
                context.stop_asap();
                return;
              }
          } catch (const std::exception& e) {
            context.mainserv.post(thrower(boost::current_exception()));
            return;
          } catch (const boost::exception& e) {
            context.mainserv.post(thrower(boost::current_exception()));
            return;
          }
          ++n_calls;
          if (respawn(n_calls)) {
            context.workserv.post(bind(&invoker::async_wait_for_input, this));
          }
          else
            ECTO_LOG_DEBUG("n_calls (%u) reached, no respawn", n_calls);
        }
        
        bool inputs_ready()
        {
          graph_t::in_edge_iterator in_beg, in_end;
          for (tie(in_beg, in_end) = in_edges(vd, g);
               in_beg != in_end; ++in_beg)
            {
              graph::edge::ptr e = g[*in_beg];
              if (e->size() == 0)
                return false;
            }

          graph_t::out_edge_iterator out_beg, out_end;
          for (tie(out_beg, out_end) = out_edges(vd, g);
               out_beg != out_end; ++out_beg)
            {
              graph::edge::ptr e = g[*out_beg];
              if (e->size() > 0)
                return false;
            }

          return true;
        }

        ~invoker() { ECTO_LOG_DEBUG("%s ~invoker", this); }
      }; // struct invoker

      void reset_times(graph_t& graph)
      {
        graph_t::vertex_iterator begin, end;
        for (tie(begin, end) = vertices(graph);
             begin != end;
             ++begin)
          {
            module::ptr m = graph[*begin];
            m->stats.ncalls = 0;
            m->stats.total_ticks = 0;
          }
      }

      static void sigint_static_thunk(int) 
      {
        std::cerr << "*** SIGINT received, stopping work queues.\n"
                  << "*** If you are stuck here, you may need to hit ^C again\n"
                  << "*** when back in the interpreter thread.\n" 
                  << "*** or Ctrl-\\ (backslash) for a hard stop.\n"
                  << std::endl;
        sigint_handler();
        PyErr_SetInterrupt();
      }
      static boost::function<void()> sigint_handler;
      

      int execute(unsigned nthreads, impl::respawn_cb_t respawn, graph_t& graph)
      {
        namespace asio = boost::asio;

        stop = false;

        workserv.reset();
        mainserv.reset();

        signal(SIGINT, &sigint_static_thunk);
        sigint_handler = boost::bind(&impl::stop_asap, this);

        //
        // initialize stats
        //
        starttime = pt::microsec_clock::universal_time();
        int64_t start_ticks = profile::read_tsc();
        reset_times(graph);

        //
        // start per-node threads
        //
        graph_t::vertex_iterator begin, end;
        for (tie(begin, end) = vertices(graph);
             begin != end;
             ++begin)
          {
            impl::invoker::ptr ip(new impl::invoker(*this, graph, *begin, respawn));
            invokers[*begin] = ip;
            workserv.post(boost::bind(&impl::invoker::async_wait_for_input, ip));
          }

        std::set<runandjoin::ptr> runners;
        for (unsigned j=0; j<nthreads; ++j)
          {
            ECTO_LOG_DEBUG("%s Start thread %u", this % j);
            runandjoin::ptr rj(new runandjoin);
            rj->run(propagator(workserv, mainserv));
            runners.insert(rj);
          }

        // run main service
        try {
          mainserv.run();
          PyErr_CheckSignals();
        } catch (const exception& e) {
          workserv.stop();
          // rethrow:  python interpreter will catch.
          throw;
        }

        //
        //  print stats
        //
        pt::time_duration elapsed = pt::microsec_clock::universal_time() - starttime;
        int64_t elapsed_ticks = profile::read_tsc() - start_ticks;

        double total_percentage = 0.0;

        std::cout << "****************************************\n";
        for (tie(begin, end) = vertices(graph); begin != end; ++begin)
          {
            module::ptr m = graph[*begin];
            double this_percentage = 100.0 * ((double)m->stats.total_ticks / elapsed_ticks);
            total_percentage += this_percentage;
            double hz = (double(m->stats.ncalls) / (elapsed.total_microseconds() / 1e+06));
            double theo_hz = hz *(100/this_percentage);
            std::cout << str(boost::format(">>> %25s  calls: %u  Hz(theo max): %3.2f Hz(real): %3.2f  cpu load: (%04.2lf%%)")
                             % m->name()
                             % m->stats.ncalls 
                             % theo_hz
                             % hz
                             % this_percentage)
                      << "\n";
          }
              
        std::cout << "**********************************************"
                  << "\ncpu freq:         " << (elapsed_ticks / (elapsed.total_milliseconds() / 1000.0)) / 1e+9 
                  << " GHz"
                  << "\nthreads:          " << nthreads
                  << "\nelapsed time:     " << elapsed 
                  << "\ncpu ticks:        " << elapsed_ticks
                  << "\ncpu ticks per second: " << elapsed.ticks_per_second();
          ;

        std::cout << str(boost::format("\nin process():     %.2f%%\n") % (total_percentage / nthreads))
          ;
        return 0;
      }

      ~impl() 
      {
        // be sure your invokers disappear before you do (you're
        // holding the main service)
        invokers_t().swap(invokers);
      }

      void stop_asap() 
      {
        stop = true;
      }

      typedef std::map<graph_t::vertex_descriptor, invoker::ptr> invokers_t;
      invokers_t invokers;
      boost::asio::io_service mainserv, workserv;
      boost::unordered_map<ecto::strand, 
                           boost::shared_ptr<boost::asio::io_service::strand>,
                           ecto::strand_hash> strands;

      pt::ptime starttime;
      bool stop;
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

    boost::function<void()> threadpool::impl::sigint_handler;
  }
}


/*
namespace {
  void forward_sigint(int) 
  {
    PyErr_SetInterrupt();
    PyErr_CheckSignals();
  }
}


signal(SIGINT, &forward_sigint);

*/

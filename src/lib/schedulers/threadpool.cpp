// 
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// 
// #define ECTO_THREADPOOL_DEBUG

#if defined(ECTO_THREADPOOL_DEBUG)
#define ECTO_LOG_ON
#define ECTO_USLEEP() usleep(500000)
#else
#define ECTO_USLEEP()
#endif

#include <ecto/ecto.hpp>

#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/cell.hpp>
#include <ecto/log.hpp>
#include <ecto/strand.hpp>

#include <ecto/graph_types.hpp>
#include <ecto/edge.hpp>
#include <ecto/schedulers/invoke.hpp>
#include <ecto/schedulers/threadpool.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>
#include <boost/format.hpp>

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

  using namespace ecto::except;

  namespace schedulers {
    namespace asio = boost::asio;

    namespace pt = boost::posix_time;

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
        ECTO_LOG_DEBUG("%s", __PRETTY_FUNCTION__);
        rethrow_exception(eptr);
      }
    };

    struct runandjoin
    {
      typedef shared_ptr<runandjoin> ptr;

      thread runner;

      runandjoin() { }

      ~runandjoin() {
        if (runner.joinable())
          runner.join();
      }

      void join() 
      {
        if (runner.joinable())
          runner.join();
      }

      void interrupt() 
      {
        ECTO_LOG_DEBUG("interrupting %p", this);
        runner.interrupt();
        ECTO_LOG_DEBUG("interrupt    %p done", this);
      }

      template <typename Work>
      void impl(Work w)
      {
        try {
          w();
        } catch (const boost::exception& e) {
          ECTO_LOG_DEBUG("post thrower(boost::exception) %s", __PRETTY_FUNCTION__);
          w.post(thrower(boost::current_exception()));
        } catch (const std::exception& e) {
          ECTO_LOG_DEBUG("post thrower (std::exception) %s", __PRETTY_FUNCTION__);
          w.post(thrower(boost::current_exception()));
        }
        w.post(bind(&runandjoin::join, this));
      }

      template <typename Work>
      void run(Work w)
      {
        boost::scoped_ptr<thread> newthread(new thread(bind(&runandjoin::impl<Work>, this, w)));
        newthread->swap(runner);
      }
    };

    struct threadpool::impl 
    {
      typedef boost::function<bool(unsigned)> respawn_cb_t;

      struct invoker : boost::noncopyable
      {
        typedef boost::shared_ptr<invoker> ptr;

        threadpool::impl& context;

        graph_t& g;
        graph_t::vertex_descriptor vd;
        unsigned n_calls;
        respawn_cb_t respawn;

        invoker(threadpool::impl& context_,
                graph_t& g_, 
                graph_t::vertex_descriptor vd_,
                respawn_cb_t respawn_)
          : context(context_), g(g_), vd(vd_), n_calls(0), respawn(respawn_)
        { }

        template <typename Handler>
        void post(Handler h)
        {
          cell::ptr m = g[vd];
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
          boost::this_thread::interruption_point();
          //ECTO_LOG_DEBUG("%s async_wait_for_input", this);
          ECTO_USLEEP();
          namespace asio = boost::asio;

          if (context.stop) return;
            
          if (inputs_ready()) {
            ECTO_LOG_DEBUG("%s inputs ready", this);
            post(bind(&invoker::invoke, this));
          } else {
            context.workserv.post(bind(&invoker::async_wait_for_input, this));
          }
        }

        void invoke()
        {
          try {
            boost::this_thread::interruption_point();
            ECTO_LOG_DEBUG("%s invoke", this);
            try {
              int rval = ecto::schedulers::invoke_process(g, vd);

              if (rval != ecto::OK)
                {
                  std::cout << "Module " << g[vd]->name() << " returned " << ReturnCodeToStr(rval) <<" Stopping everything."
                            << std::endl; 
                  context.stop = true;
                  //        context.interrupt();
                  context.mainserv.post(boost::bind(&threadpool::impl::interrupt, &context));
                  return;
                }
            } catch (const std::exception& e) {
              ECTO_LOG_DEBUG("post thrower std::exception %s", __PRETTY_FUNCTION__);
              context.mainserv.post(thrower(boost::current_exception()));
              return;
            } catch (const boost::exception& e) {
              ECTO_LOG_DEBUG("post thrower boost::exception %s", __PRETTY_FUNCTION__);
              context.mainserv.post(thrower(boost::current_exception()));
              return;
            }
            ++n_calls;
            if (respawn(n_calls)) {
              context.workserv.post(bind(&invoker::async_wait_for_input, this));
            }
            else
              ECTO_LOG_DEBUG("n_calls (%u) reached, no respawn", n_calls);
          } catch (const boost::thread_interrupted&) {
            context.stop = true;
            context.mainserv.post(boost::bind(&threadpool::impl::interrupt, &context));
          }
        }
        
        bool inputs_ready()
        {
          graph_t::in_edge_iterator in_beg, in_end;
          for (tie(in_beg, in_end) = in_edges(vd, g);
               in_beg != in_end; ++in_beg)
            {
              graph::edge_ptr e = g[*in_beg];
              if (e->size() == 0)
                return false;
            }

          graph_t::out_edge_iterator out_beg, out_end;
          for (tie(out_beg, out_end) = out_edges(vd, g);
               out_beg != out_end; ++out_beg)
            {
              graph::edge_ptr e = g[*out_beg];
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
            cell::ptr m = graph[*begin];
            m->stats.ncalls = 0;
            m->stats.total_ticks = 0;
          }
      }

      static boost::signals2::signal<void(void)> THREADPOOL_SIGINT_SIGNAL;
      static void sigint_static_thunk(int)
      {
        std::cerr << "*** SIGINT received, stopping graph execution.\n"
                  << "*** If you are stuck here, you may need to hit ^C again\n"
                  << "*** when back in the interpreter thread.\n" << "*** or Ctrl-\\ (backslash) for a hard stop.\n"
                  << std::endl;
        THREADPOOL_SIGINT_SIGNAL();
        PyErr_SetInterrupt();
      }

      int execute(unsigned nthreads, impl::respawn_cb_t respawn, graph_t& graph)
      {
        namespace asio = boost::asio;

        boost::mutex::scoped_lock running(running_mtx);
        stop = false; //reset the stop
        workserv.reset();
        mainserv.reset();
        boost::signals2::scoped_connection interupt_connection(
            THREADPOOL_SIGINT_SIGNAL.connect(boost::bind(&impl::stop_asap, this)));
#if !defined(_WIN32)
        signal(SIGINT, &sigint_static_thunk);
#endif
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
        } catch (const boost::exception& e) {
          workserv.stop();
          throw;
        }

        //
        //  print stats
        //
        pt::time_duration elapsed = pt::microsec_clock::universal_time() - starttime;
        int64_t elapsed_ticks = profile::read_tsc() - start_ticks;

        double total_percentage = 0.0;

        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << str(boost::format("* %25s   %-7s %-10s %-10s %-6s\n") % "Cell Name" % "Calls" % "Hz(theo max)" % "Hz(observed)" % "load (%)");
        for (tie(begin, end) = vertices(graph); begin != end; ++begin)
          {
            cell::ptr m = graph[*begin];
            double this_percentage = 100.0 * ((double)m->stats.total_ticks / elapsed_ticks);
            total_percentage += this_percentage;
            double hz = (double(m->stats.ncalls) / (elapsed.total_microseconds() / 1e+06));
            double theo_hz = hz *(100/this_percentage);
            std::cout << str(boost::format("* %25s   %-7u %-12.2f %-12.2f %-8.2lf")
                             % m->name()
                             % m->stats.ncalls 
                             % theo_hz
                             % hz
                             % this_percentage)
                      << "\n";
          }
              
        std::cout << "------------------------------------------------------------------------------"
                  << "\ncpu freq:         " << (elapsed_ticks / (elapsed.total_milliseconds() / 1000.0)) / 1e+9 
                  << " GHz"
                  << "\nthreads:          " << nthreads
                  << "\nelapsed time:     " << elapsed 
                  << "\ncpu ticks:        " << elapsed_ticks
                  << "\ncpu ticks/second: " << elapsed.ticks_per_second();
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

      impl()
      {
        stop = false;
      }

      bool running() 
      {
        boost::mutex::scoped_lock lock(running_mtx, boost::defer_lock);
        return !lock.try_lock();
      }

      void interrupt()
      {
        stop = true;
        mainserv.stop();
        workserv.stop();
        for (std::set<runandjoin::ptr>::iterator iter = runners.begin();
             iter != runners.end(); ++iter)
          {
            (*iter)->interrupt();
          }
      }

      void join()
      {
        for (std::set<runandjoin::ptr>::iterator iter = runners.begin();
             iter != runners.end(); ++iter)
          {
            (*iter)->join();
          }
      }

    private:

      typedef std::map<graph_t::vertex_descriptor, invoker::ptr> invokers_t;
      invokers_t invokers;
      boost::asio::io_service mainserv, workserv;
      boost::unordered_map<ecto::strand, 
                           boost::shared_ptr<boost::asio::io_service::strand>,
                           ecto::strand_hash> strands;

      std::set<runandjoin::ptr> runners;
      pt::ptime starttime;
      bool stop;
      
      boost::mutex running_mtx;

    };

    //////////////////////////////////////////////////////////////
    // the outward-facing class
    //////////////////////////////////////////////////////////////


    threadpool::threadpool(plasm_ptr p)
      : plasm_(p), graph(p->graph()), impl_(new impl)
    { 
      ECTO_LOG_DEBUG("%s", __PRETTY_FUNCTION__);
    }

    threadpool::threadpool(plasm& p)
      : plasm_(p.shared_from_this()), graph(plasm_->graph()), impl_(new impl)
    { 
      ECTO_LOG_DEBUG("%s", __PRETTY_FUNCTION__);
    }

    threadpool::~threadpool()
    {
      ECTO_LOG_DEBUG("%s", __PRETTY_FUNCTION__);
      //      if (runthread.joinable())
      if (running())
        {
          std::cerr << "*** YOU ARE ATTEMPTING TO DESTROY A RUNNING SCHEDULER  ***\n"
                    << "*** I can't throw, as I'm in a destructor.             ***\n"
                    << "*** You should stop() and wait() on this schedulers.   ***\n"
                    << "*** I'm going to interrupt() things...                 ***\n";
        }
      interrupt();
      wait();
    }
    namespace phx = boost::phoenix;

    int threadpool::execute_impl(unsigned ncalls, unsigned nthreadsarg)
    {
      ECTO_LOG_DEBUG("%s", __PRETTY_FUNCTION__);

      unsigned nthreads = 
        nthreadsarg == 0 
        ? std::min((unsigned)plasm_->size(), boost::thread::hardware_concurrency())
        : nthreadsarg;

      //check this plasm for correctness.
      plasm_->check();
      plasm_->configure_all();

      std::cout << "Threadpool executing ";

      if (ncalls == 0)
        std::cout << "[unlimited]";
      else
        std::cout << ncalls;
    
      std::cout << " ticks in " << nthreads << " threads.\n";

      if (ncalls == 0)
        return impl_->execute(nthreads, phx::val(true), graph);
      else
        return impl_->execute(nthreads, boost::phoenix::arg_names::arg1 < ncalls, graph);
    }

    int threadpool::execute(unsigned ncalls, unsigned nthreadsarg)
    {
      boost::mutex::scoped_lock lock(iface_mtx);
      PyEval_InitThreads();
      assert(PyEval_ThreadsInitialized());
      ECTO_LOG_DEBUG("%s", __PRETTY_FUNCTION__);
      if (impl_->running())
        BOOST_THROW_EXCEPTION(EctoException()
                              << diag_msg("threadpool scheduler already running"));
      int j;
      j = execute_impl(ncalls, nthreadsarg);
      return j;
    }

    void threadpool::execute_async(unsigned ncalls, unsigned nthreadsarg)
    {
      boost::mutex::scoped_lock lock(iface_mtx);
      PyEval_InitThreads();
      assert(PyEval_ThreadsInitialized());
      
      ECTO_LOG_DEBUG("%s", __PRETTY_FUNCTION__);

      if (impl_->running())
        BOOST_THROW_EXCEPTION(EctoException()
                              << diag_msg("threadpool scheduler already running"));

      if (runthread.joinable())
        BOOST_THROW_EXCEPTION(EctoException() 
                              << diag_msg("Attempt to execute_async on unjoined schedulers... call wait() if you want to execute again"));

      boost::function<void()> fn = boost::bind(&threadpool::execute_impl, this, ncalls, nthreadsarg);

      boost::thread tmp(fn);
      tmp.swap(runthread);

      // spin until e's locked
      ECTO_LOG_DEBUG("%s", "waiting for running() to become true");
      while(!impl_->running())
        {
          usleep(10);
        }
    }

    void threadpool::stop()
    {
      boost::mutex::scoped_lock lock(iface_mtx);
      ECTO_LOG_DEBUG("%s", __PRETTY_FUNCTION__);
      impl_->stop_asap();
    }

    void threadpool::wait()
    {
      boost::mutex::scoped_lock lock(iface_mtx);
      ECTO_LOG_DEBUG("%s", __PRETTY_FUNCTION__);
      impl_->join();
      if (runthread.joinable())
        runthread.join();
    }

    void threadpool::interrupt()
    {
      boost::mutex::scoped_lock lock(iface_mtx);
      std::cout << "threadpool attempting to interrupt threads.\n";
      impl_->interrupt();
      runthread.interrupt();
    }

    bool threadpool::running() const
    {
      boost::mutex::scoped_lock lock(iface_mtx);
      ECTO_LOG_DEBUG("%s", __PRETTY_FUNCTION__);
      return impl_->running();
    }

    boost::signals2::signal<void(void)> threadpool::impl::THREADPOOL_SIGINT_SIGNAL;
  }
}



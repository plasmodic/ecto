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
// #define ECTO_TRACE_EXCEPTIONS
#define ECTO_LOG_ON
#define DISABLE_SHOW
#include <ecto/util.hpp>
#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/cell.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

#include <ecto/impl/graph_types.hpp>
#include <ecto/plasm.hpp>
#include <ecto/impl/invoke.hpp>
#include <ecto/impl/schedulers/access.hpp>
#include <ecto/schedulers/multithreaded.hpp>

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/asio.hpp>

#include <ecto/plasm.hpp>
#include <ecto/scheduler.hpp>
#include <ecto/tendril.hpp>
#include <ecto/cell.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>



namespace ecto {

  namespace schedulers {
    
    struct ECTO_EXPORT multithreaded::impl : scheduler<multithreaded::impl>
    {
      explicit impl(plasm_ptr);
      ~impl();

      int execute(unsigned niter, unsigned nthread);
      void execute_async(unsigned niter, unsigned nthread);

      void stop();
      void interrupt();
      bool running() const;
      void wait();

    private:

      int run_graph();
      int execute_impl(unsigned niter, unsigned nthread);

      int invoke_process(ecto::graph::graph_t::vertex_descriptor vd);
      void compute_stack();

      boost::thread runthread;
      
      bool stop_running;
      std::vector<ecto::graph::graph_t::vertex_descriptor> stack;
      mutable boost::mutex iface_mtx;
      mutable boost::mutex running_mtx;
      
      boost::asio::io_service serv;
      boost::mutex current_iter_mtx;
      unsigned current_iter;
    };
  }

  using namespace ecto::graph;
  using boost::scoped_ptr;
  using boost::thread;
  using boost::bind;

  namespace pt = boost::posix_time;

  namespace schedulers {

    //
    // forwarding interface
    // 

    multithreaded::multithreaded(plasm_ptr p) 
      : impl_(new impl(p))
    { }

    multithreaded::~multithreaded() { }

    int multithreaded::execute(unsigned niter, unsigned nthread)
    {
      return impl_->execute(niter, nthread);
    }

    void multithreaded::execute_async(unsigned niter, unsigned nthread)
    {
      impl_->execute_async(niter, nthread);
    }

    void multithreaded::stop() {
      impl_->stop();
    }

    bool multithreaded::running() const {
      return impl_->running();
    }

    void multithreaded::wait() {
      impl_->wait();
    }

    void multithreaded::interrupt() {
      impl_->interrupt();
    }

    std::string multithreaded::stats() {
      return impl_->stats();
    }
    
    //
    //  impl
    //


    multithreaded::impl::impl(plasm_ptr p) 
      : scheduler<multithreaded::impl>(p), stop_running(false)
    { }

    multithreaded::impl::~impl()
    {
      interrupt();
      wait();
    }


    int
    multithreaded::impl::invoke_process(graph_t::vertex_descriptor vd)
    {
      int rv;
      try {
        rv = ecto::schedulers::invoke_process(graph, vd);
      } catch (const boost::thread_interrupted& e) {
        std::cout << "Interrupted\n";
        return ecto::QUIT;
      }
      return rv;
    }

    void multithreaded::impl::compute_stack()
    {
      if (!stack.empty()) //will be empty if this needs to be computed.
        return;
      //check this plasm for correctness.
      plasm_->check();
      plasm_->configure_all();
      boost::topological_sort(graph, std::back_inserter(stack));
      std::reverse(stack.begin(), stack.end());
    }

    namespace
    {
      boost::signals2::signal<void(void)> SINGLE_THREADED_SIGINT_SIGNAL;
      void
      sigint_static_thunk(int)
      {
        std::cerr << "*** SIGINT received, stopping graph execution.\n"
                  << "*** If you are stuck here, you may need to hit ^C again\n"
                  << "*** when back in the interpreter thread.\n" << "*** or Ctrl-\\ (backslash) for a hard stop.\n"
                  << std::endl;
        SINGLE_THREADED_SIGINT_SIGNAL();
        PyErr_SetInterrupt();
      }
    }

    void multithreaded::impl::execute_async(unsigned niter, unsigned nthread)
    {
      PyEval_InitThreads();
      assert(PyEval_ThreadsInitialized());

      boost::mutex::scoped_lock l(iface_mtx);
      // compute_stack(); //FIXME hack for python based tendrils.
      scoped_ptr<thread> tmp(new thread(bind(&multithreaded::impl::execute_impl, 
                                             this, niter, nthread)));
      tmp->swap(runthread);
      while(!running()) 
        boost::this_thread::sleep(boost::posix_time::microseconds(5)); //TODO FIXME condition variable?
    }

    int multithreaded::impl::execute(unsigned niter, unsigned nthread)
    {
      int j;
      j = execute_impl(niter, nthread);
      return j;
    }

    struct stack_runner 
    {
      graph::graph_t& graph;
      const std::vector<graph::graph_t::vertex_descriptor>& stack;
      boost::asio::io_service& serv;
      unsigned max_iter;
      unsigned& overall_current_iter;
      boost::mutex& overall_current_iter_mtx;

      stack_runner(graph::graph_t& graph_,
                   const std::vector<graph::graph_t::vertex_descriptor>& stack_, 
                   boost::asio::io_service& serv_,
                   unsigned max_iter_,
                   unsigned& overall_current_iter_,
                   boost::mutex& overall_current_iter_mtx_)
        : graph(graph_), 
          stack(stack_), 
          serv(serv_), 
          max_iter(max_iter_), 
          overall_current_iter(overall_current_iter_),
          overall_current_iter_mtx(overall_current_iter_mtx_)
      { 
        ECTO_LOG_DEBUG("Created stack_runner @ overall iteration %u", overall_current_iter);
      }
      
      typedef int result_type;

      result_type operator()(std::size_t index)
      {
        ECTO_LOG_DEBUG("Runner firing on index %u of %u", index % stack.size());
        size_t retval = invoke_process(graph, stack[index]);
        ++index;
        assert (index <= stack.size());
        if (index == stack.size()) {
          boost::mutex::scoped_lock lock(overall_current_iter_mtx);
          ECTO_LOG_DEBUG("Thread deciding whether to recycle @ index %u, overall iter=%u", 
                         index % overall_current_iter);
          index = 0;
          if (overall_current_iter == max_iter)
            {
              ECTO_LOG_DEBUG("Thread exiting at %u iterations", max_iter);
              return 0;
            }
          else
            {
              ++overall_current_iter;
            }
        }
        ECTO_LOG_DEBUG("Posting next job index=%u", index);
        serv.post(boost::bind(stack_runner(graph, stack, serv, 
                                           max_iter, 
                                           overall_current_iter, overall_current_iter_mtx), index));
        return retval;
      }
    };


    int multithreaded::impl::run_graph()
    {
      return 0;
    }

    int multithreaded::impl::execute_impl(unsigned max_iter, unsigned nthread)
    {
      ECTO_LOG_DEBUG("execute_impl max_iter=%u nthread=%u",
                     max_iter % nthread);
      plasm_->reset_ticks();
      compute_stack();
      boost::mutex::scoped_lock yes_running(running_mtx);
      stop_running = false;

      boost::signals2::scoped_connection 
        interupt_connection(SINGLE_THREADED_SIGINT_SIGNAL.connect(boost::bind(&multithreaded::impl::interrupt, this)));

#if !defined(_WIN32)
      signal(SIGINT, &sigint_static_thunk);
#endif
      
      profile::graphstats_collector gs(graphstats);

      if (max_iter < nthread) {
        nthread = max_iter;
        ECTO_LOG_DEBUG("Clamped threads to %u", nthread);
      }
      for (unsigned j=0; j<nthread; ++j)
        serv.post(boost::bind(stack_runner(graph, stack, serv, 
                                           max_iter, 
                                           current_iter,
                                           current_iter_mtx), 
                              0));

      boost::thread_group threads;

      for (unsigned j=0; j<nthread; ++j)
        {
          ECTO_LOG_DEBUG("Running service in thread %u", j);
          threads.create_thread(boost::bind(&boost::asio::io_service::run, &serv));
          boost::mutex::scoped_lock lock(current_iter_mtx);
          ++current_iter;
        }
      threads.join_all();
      ECTO_LOG_DEBUG("JOINED, EXITING AFTER %u", current_iter);
      return stop_running;
    }

    void multithreaded::impl::interrupt() {
      boost::mutex::scoped_lock l(iface_mtx);
      SHOW();
      stop_running = true;
      runthread.interrupt();
      runthread.join();
    }

    void multithreaded::impl::stop() {
      boost::mutex::scoped_lock l(iface_mtx);
      SHOW();
      stop_running = true;
      runthread.join();
    }

    bool multithreaded::impl::running() const {
      boost::mutex::scoped_lock l2(running_mtx, boost::defer_lock);
      if (l2.try_lock())
        return false;
      else
        return true;
    }


    void multithreaded::impl::wait() {
      SHOW();
      boost::mutex::scoped_lock l(iface_mtx);
      while(running())
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
      runthread.join();
    }


  }
}


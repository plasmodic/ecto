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
#include <ecto/schedulers/multithreaded.hpp>

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/graph/topological_sort.hpp>

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

      int execute(unsigned niter=0);
      void execute_async(unsigned niter=0);

      void stop();
      void interrupt();
      bool running() const;
      void wait();

    private:

      int execute_impl(unsigned niter);

      int invoke_process(ecto::graph::graph_t::vertex_descriptor vd);
      void compute_stack();

      boost::thread runthread;
      
      bool stop_running;
      int last_rval;
      std::vector<ecto::graph::graph_t::vertex_descriptor> stack;
      mutable boost::mutex iface_mtx;
      mutable boost::mutex running_mtx;
    };
  }

  using namespace ecto::graph;
  using boost::scoped_ptr;
  using boost::thread;
  using boost::bind;

  namespace pt = boost::posix_time;

  namespace schedulers {

    multithreaded::multithreaded(plasm_ptr p) 
      : impl_(new impl(p))
    { }

    multithreaded::impl::impl(plasm_ptr p) 
      : scheduler<multithreaded::impl>(p), stop_running(false)
    { }

    multithreaded::~multithreaded() { }

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

    void multithreaded::execute_async(unsigned niter) 
    {
      impl_->execute_async(niter);
    }

    void multithreaded::impl::execute_async(unsigned niter)
    {
      PyEval_InitThreads();
      assert(PyEval_ThreadsInitialized());

      boost::mutex::scoped_lock l(iface_mtx);
      // compute_stack(); //FIXME hack for python based tendrils.
      scoped_ptr<thread> tmp(new thread(bind(&multithreaded::impl::execute_impl, 
                                             this, niter)));
      tmp->swap(runthread);
      while(!running()) 
        boost::this_thread::sleep(boost::posix_time::microseconds(5)); //TODO FIXME condition variable?
    }

    int multithreaded::execute(unsigned niter)
    {
      return impl_->execute(niter);
    }

    int multithreaded::impl::execute(unsigned niter)
    {
      int j;
      j = execute_impl(niter);
      return j;
    }

    int multithreaded::impl::execute_impl(unsigned niter)
    {
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

      unsigned cur_iter = 0;
      while((niter == 0 || cur_iter < niter) && !stop_running)
        {
          for (size_t k = 0; k < stack.size() && !stop_running; ++k)
            {
              //need to check the return val of a process here, non zero means exit...
              size_t retval = invoke_process(stack[k]);
              last_rval = retval;
              if (retval)
                return retval;
            }
          ++cur_iter;
        }
      last_rval = stop_running;
      return last_rval;
    }

    void multithreaded::interrupt() {
      impl_->interrupt();
    }
    void multithreaded::impl::interrupt() {
      boost::mutex::scoped_lock l(iface_mtx);
      SHOW();
      stop_running = true;
      runthread.interrupt();
      runthread.join();
    }

    void multithreaded::stop() {
      impl_->stop();
    }
    void multithreaded::impl::stop() {
      boost::mutex::scoped_lock l(iface_mtx);
      SHOW();
      stop_running = true;
      runthread.join();
    }

    bool multithreaded::running() const {
      return impl_->running();
    }

    bool multithreaded::impl::running() const {
      boost::mutex::scoped_lock l2(running_mtx, boost::defer_lock);
      if (l2.try_lock())
        return false;
      else
        return true;
    }

    void multithreaded::wait() {
      impl_->wait();
    }

    void multithreaded::impl::wait() {
      SHOW();
      boost::mutex::scoped_lock l(iface_mtx);
      while(running())
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
      runthread.join();
    }

    std::string multithreaded::stats() {
      return impl_->stats();
    }

  }
}


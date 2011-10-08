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

#include <ecto/graph_types.hpp>
#include <ecto/plasm.hpp>
#include <ecto/schedulers/invoke.hpp>
#include <ecto/schedulers/singlethreaded.hpp>

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>


namespace ecto {

  using namespace ecto::graph;
  using boost::scoped_ptr;
  using boost::thread;
  using boost::bind;

  namespace schedulers {

    singlethreaded::singlethreaded(plasm_ptr p)
      : plasm_(p), graph(p->graph()),stop_running(false)
    {
      assert(plasm_);
    }

    singlethreaded::singlethreaded(plasm& p)
      : plasm_(p.shared_from_this()), graph(plasm_->graph()),stop_running(false)
    {
      assert(plasm_);
    }

    singlethreaded::~singlethreaded()
    {
      interrupt();
      wait();
    }
    int
    singlethreaded::invoke_process(graph_t::vertex_descriptor vd)
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

    void singlethreaded::compute_stack()
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

    void singlethreaded::execute_async(unsigned niter)
    {
      PyEval_InitThreads();
      assert(PyEval_ThreadsInitialized());

      boost::mutex::scoped_lock l(iface_mtx);
      // compute_stack(); //FIXME hack for python based tendrils.
      scoped_ptr<thread> tmp(new thread(bind(&singlethreaded::execute_impl, this, niter)));
      tmp->swap(runthread);
      while(!running()) boost::this_thread::sleep(boost::posix_time::microseconds(5)); //TODO FIXME condition variable?
    }

    int singlethreaded::execute(unsigned niter)
    {
      int j;
      j = execute_impl(niter);
      return j;
    }

    int singlethreaded::execute_impl(unsigned niter)
    {
      compute_stack();
      boost::mutex::scoped_lock yes_running(running_mtx);

      stop_running = false;
      boost::signals2::scoped_connection interupt_connection(
          SINGLE_THREADED_SIGINT_SIGNAL.connect(boost::bind(&singlethreaded::interrupt, this)));
      #if !defined(_WIN32)
            signal(SIGINT, &sigint_static_thunk);
      #endif

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

    void singlethreaded::interrupt() {
      boost::mutex::scoped_lock l(iface_mtx);
      SHOW();
      stop_running = true;
      runthread.interrupt();
      runthread.join();
    }

    void singlethreaded::stop() {
      boost::mutex::scoped_lock l(iface_mtx);
      SHOW();
      stop_running = true;
      runthread.join();
    }

    bool singlethreaded::running() const {
      boost::mutex::scoped_lock l2(running_mtx, boost::defer_lock);
      if (l2.try_lock())
        return false;
      else
        return true;
    }

    void singlethreaded::wait() {
      SHOW();
      boost::mutex::scoped_lock l(iface_mtx);
      while(running())
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
      runthread.join();
    }

  }
}


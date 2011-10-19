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
//#define DISABLE_SHOW
#define ECTO_LOG_ON
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
#include <ecto/schedulers/singlethreaded.hpp>

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

  using namespace ecto::except;

  namespace schedulers {
    
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

    using namespace ecto::graph;
    using boost::thread;
    using boost::bind;

    namespace pt = boost::posix_time;

    using boost::scoped_ptr;

    singlethreaded::singlethreaded(plasm_ptr p) 
      : scheduler(p)
    { }

    singlethreaded::~singlethreaded() 
    { 
      interrupt();
      //      wait_for_running_is(false);
      // wait();
    }

    int singlethreaded::execute_impl(unsigned niter, unsigned nthread, boost::asio::io_service& topserv)
    {
      ECTO_START();
      plasm_->reset_ticks();
      compute_stack();

      boost::signals2::scoped_connection 
        interupt_connection(SINGLE_THREADED_SIGINT_SIGNAL.connect(boost::bind(&singlethreaded::interrupt, this)));

#if !defined(_WIN32)
      signal(SIGINT, &sigint_static_thunk);
#endif
      
      profile::graphstats_collector gs(graphstats);

      unsigned cur_iter = 0;
      while((niter == 0 || cur_iter < niter) && !stop_running)
        {
          for (size_t k = 0; k < stack.size(); ++k)
            {
              ECTO_LOG_DEBUG("stop_running=%d k=%u niter=%u", stop_running % k % niter);
              //need to check the return val of a process here, non zero means exit...
              size_t retval = invoke_process(stack[k]);
              last_rval = retval;
              if (retval) {
                stop_running = true;
                return retval;
              }
            }
          ++cur_iter;
        }
      ECTO_LOG_DEBUG("FINISH %s", __PRETTY_FUNCTION__);
      last_rval = stop_running;
      return last_rval;
    }

    void singlethreaded::interrupt_impl() 
    {
      SHOW();
      stop_running = true;
      runthread.interrupt();
      runthread.join();
      running(false);
    }

    void singlethreaded::stop_impl() 
    {
      SHOW();
    }
    void singlethreaded::wait_impl() 
    {
      SHOW();
      runthread.join();
      //      wait_for_running_is(false);
    }
  }
}


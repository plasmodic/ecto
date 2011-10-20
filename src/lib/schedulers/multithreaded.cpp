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

#include <ecto/util.hpp>
#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/cell.hpp>
#include <ecto/rethrow.hpp>

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
#include <boost/asio.hpp>

#include <ecto/plasm.hpp>
#include <ecto/scheduler.hpp>
#include <ecto/tendril.hpp>
#include <ecto/cell.hpp>
#include <ecto/except.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

namespace ecto {

  using namespace ecto::except;

  namespace schedulers {

    using namespace ecto::graph;
    using boost::scoped_ptr;
    using boost::thread;
    using boost::bind;

    namespace pt = boost::posix_time;


    //
    // forwarding interface
    //

    multithreaded::multithreaded(plasm_ptr p)
      : scheduler(p),
        current_iter(0)
    { }

    multithreaded::~multithreaded()
    {
      if (!running())
        return;
      interrupt();
      wait();
    }

    //
    //  impl
    //

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

    struct stack_runner
    {
      graph::graph_t& graph;
      const std::vector<graph::graph_t::vertex_descriptor>& stack;
      boost::asio::io_service& serv;
      unsigned max_iter;
      unsigned& overall_current_iter;
      boost::mutex& overall_current_iter_mtx;
      boost::asio::io_service& topserv;

      stack_runner(graph::graph_t& graph_,
                   const std::vector<graph::graph_t::vertex_descriptor>& stack_,
                   boost::asio::io_service& serv_,
                   unsigned max_iter_,
                   unsigned& overall_current_iter_,
                   boost::mutex& overall_current_iter_mtx_,
                   boost::asio::io_service& topserv_)
        : graph(graph_),
          stack(stack_),
          serv(serv_),
          max_iter(max_iter_),
          overall_current_iter(overall_current_iter_),
          overall_current_iter_mtx(overall_current_iter_mtx_),
          topserv(topserv_)
      {
        ECTO_LOG_DEBUG("Created stack_runner @ overall iteration %u, max %u",
                       overall_current_iter % max_iter);
      }

      typedef int result_type;

      result_type operator()(std::size_t index)
      {
        cell::ptr m = graph[stack[index]];
        access cellaccess(*m);
        boost::mutex::scoped_lock lock(cellaccess.mtx);

        ECTO_LOG_DEBUG("Runner firing on index %u of %u cell %s", index % stack.size() % m->name());

        size_t retval = invoke_process(graph, stack[index]);
        if (retval != ecto::OK)
          {
            cellaccess.stop_requested = true;
            return retval;
          }
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
        boost::function<void()> f = boost::bind(stack_runner(graph, stack, serv,
                                                             max_iter,
                                                             overall_current_iter, overall_current_iter_mtx,
                                                             topserv),
                                                index);
        serv.post(boost::bind(&ecto::except::py::rethrow, f, boost::ref(topserv)));
        return retval;
      }
    };

    int multithreaded::execute_impl(unsigned max_iter, unsigned nthread, boost::asio::io_service& topserv)
    {
      ECTO_LOG_DEBUG("execute_impl max_iter=%u nthread=%u",
                     max_iter % nthread);

      plasm_->reset_ticks();
      compute_stack();
      current_iter = 0;

      serv.reset();

      boost::signals2::scoped_connection
        interupt_connection(SINGLE_THREADED_SIGINT_SIGNAL.connect(boost::bind(&multithreaded::interrupt_impl, this)));

#if !defined(_WIN32)
      signal(SIGINT, &sigint_static_thunk);
#endif

      profile::graphstats_collector gs(graphstats);

      if (max_iter > 0 && max_iter < nthread) {
        nthread = max_iter;
        ECTO_LOG_DEBUG("Clamped threads to %u", nthread);
      }
      for (unsigned j=0; j<nthread; ++j)
        {
          boost::function<void()> f = boost::bind(stack_runner(graph, stack, serv,
                                                               max_iter,
                                                               current_iter, current_iter_mtx,
                                                               topserv),
                                                  0);
          serv.post(boost::bind(&ecto::except::py::rethrow, f, boost::ref(topserv)));
        }
      for (unsigned j=0; j<nthread; ++j)
        {
          ECTO_LOG_DEBUG("Running service in thread %u", j);
          boost::function<void()> runit = boost::bind(&verbose_run, boost::ref(serv),
                                                      str(boost::format("worker_%u") % j));
          threads.create_thread(boost::bind(&ecto::except::py::rethrow, runit, boost::ref(topserv)));
          boost::mutex::scoped_lock lock(current_iter_mtx);
          ++current_iter;
        }

      verbose_run(topserv, "topserv");
      threads.join_all();
      ECTO_LOG_DEBUG("JOINED, EXITING AFTER %u", current_iter);

      this->running(false);
      return 0;
    }

    void multithreaded::interrupt_impl() {
      stop();
      threads.join_all();
      runthread.interrupt();
      runthread.join();
      running(false);
    }

    void multithreaded::stop_impl()
    {
    }

    void multithreaded::wait_impl()
    {
      while(running())
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
      threads.join_all();
      runthread.join();
    }
  }
}



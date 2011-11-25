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
#include <boost/format.hpp>

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

    struct stack_runner
    {
      multithreaded& ctx;
      unsigned max_iter;
      boost::asio::io_service::work topwork;

      stack_runner(multithreaded& ctx_,
                   unsigned max_iter_)
        : ctx(ctx_),
          max_iter(max_iter_),
          topwork(ctx.top_serv)
      {
        ECTO_LOG_DEBUG("Created stack_runner @ overall iteration %u, max %u, workserv=%p",
                       ctx.current_iter.get() % max_iter % &ctx.workserv);
      }

      typedef int result_type;

      result_type operator()(std::size_t index)
      {
        ECTO_ASSERT(index < ctx.stack.size(), "index out of bounds");
        cell::ptr m = ctx.graph[ctx.stack[index]];
        access cellaccess(*m);
        ECTO_LOG_DEBUG("Runner firing on cell %u/%u (%s) iter %u",
                       index % ctx.stack.size() % m->name() % ctx.current_iter.get());
        boost::mutex::scoped_lock lock(cellaccess.mtx);
        ECTO_LOG_DEBUG("Runner LOCKED on cell %u/%u (%s) iter %u",
                       index % ctx.stack.size() % m->name() % ctx.current_iter.get());

        //
        //  TDS just use multithreaded as context, nix the rethrow?
        //
        size_t retval = invoke_process(ctx.graph, ctx.stack[index]);

        if (retval != ecto::OK)
          {
            cellaccess.stop_requested = true;
            return retval;
          }
        ++index;
        ECTO_ASSERT (index <= ctx.stack.size(), "index out of bounds");
        {
          ecto::atomic<unsigned>::scoped_lock oci(ctx.current_iter);
          if (index == ctx.stack.size())
            {

              ECTO_LOG_DEBUG("Thread deciding whether to recycle @ index %u, overall iter=%u of %u",
                             index % oci.value % max_iter);
              index = 0;
              ECTO_ASSERT(max_iter == 0 || oci.value <= max_iter, "uh oh timing gone bad");
              if (max_iter && oci.value >= max_iter)
              {
                ECTO_LOG_DEBUG("Thread exiting at %u iterations", max_iter);
                return 0;
              }
            else
              {
                ++oci.value;
              }
          }
          ECTO_LOG_DEBUG("Posting next job index=%u", index);
          boost::function<void()> f = boost::bind(stack_runner(ctx,
                                                               max_iter),
                                                  index);
          on_strand(ctx.graph[ctx.stack[index]], ctx.workserv, boost::bind(&ecto::except::py::rethrow, f,
                                                 boost::ref(ctx.top_serv), &ctx));
          return retval;
        }
      }
    };

    int multithreaded::execute_impl(unsigned max_iter, unsigned nthread, boost::asio::io_service&)
    {
      ECTO_LOG_DEBUG("execute_impl max_iter=%u nthread=%u",
                     max_iter % nthread);

      {
        ecto::atomic<unsigned>::scoped_lock l(current_iter);
        l.value = 0;
      }
      workserv.reset();

      profile::graphstats_collector gs(graphstats);

      if (max_iter > 0 && max_iter < nthread) {
        nthread = max_iter;
        ECTO_LOG_DEBUG("Clamped threads to %u", nthread);
      }
      for (unsigned j=0; j<nthread; ++j)
        {
          ECTO_LOG_DEBUG("Creating initial stack runner %u of %u", j % nthread);
          boost::function<void()> f = boost::bind(stack_runner(*this,
                                                               max_iter),
                                                  0);

          on_strand(graph[stack[0]], workserv, boost::bind(&ecto::except::py::rethrow, f,
                                                           boost::ref(top_serv), this));
        }

      ECTO_LOG_DEBUG("Spawning %u workserv runner threads", nthread);
      {
        ecto::atomic<unsigned>::scoped_lock oci(current_iter);

        for (unsigned j=0; j<nthread; ++j)
          {
            ECTO_LOG_DEBUG("Running service in thread %u", j);
            std::string thisname = str(boost::format("worker_%u") % j);
            boost::function<void()> runit = boost::bind(&verbose_run, boost::ref(workserv), thisname);
            threads.create_thread(boost::bind(&ecto::except::py::rethrow, runit,
                                              boost::ref(top_serv), this));
            ++oci.value;
          }
      }
      verbose_run(top_serv, "top_serv");

      threads.join_all();
      {
        ecto::atomic<unsigned>::scoped_lock oci(current_iter);
        ECTO_LOG_DEBUG("JOINED, EXITING AFTER %u of %u", oci.value % max_iter);
        ECTO_ASSERT(max_iter == 0 || oci.value <= max_iter, "uh oh, our timing is way off");
      }
      //reset all of the strands that may have references to our ioservice and what not.
      for(size_t i = 0; i < stack.size(); i ++)
      {
        if(graph[stack[i]]->strand_){
          graph[stack[i]]->strand_->reset();
        }
      }
      return 0;
    }

    void multithreaded::interrupt_impl() {
      stop();
      threads.interrupt_all();
      threads.join_all();
    }

    void multithreaded::stop_impl()
    {
    }

    void multithreaded::wait_impl()
    {
      threads.join_all();
    }
  }
}



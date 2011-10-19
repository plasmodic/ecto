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

#define ECTO_LOG_ON
#include <ecto/log.hpp>

#include <ecto/cell.hpp>
#include <ecto/scheduler.hpp>
#include <ecto/impl/invoke.hpp>
#include <boost/thread.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/scoped_ptr.hpp>

namespace ecto {

  using boost::scoped_ptr;
  using boost::thread;
  using boost::bind;
  using boost::mutex;
  using boost::recursive_mutex;

  using namespace ecto::except;
  using ecto::graph::graph_t;

  scheduler::scheduler(plasm_ptr p)
    : plasm_(p)
    , graph(p->graph())
    , running_value(false)
  { }

  scheduler::~scheduler()
  {
    // don't call wait() here... you'll thunk to the virtual wait_impl
    // which will dispatch to a child class instance that no longer exists.
    // do this in the destructor of the child scheduler classes
  }

  std::string scheduler::stats()
  {
    return graphstats.as_string(graph);
  }

  void scheduler::notify_start()
  {
    assert(stack.size() > 0);
    for (unsigned j=0; j<stack.size(); ++j)
      {
        cell::ptr c = graph[stack[j]];
        c->start();
      }
  }

  void scheduler::notify_stop()
  {
    assert(stack.size() > 0);
    for (unsigned j=0; j<stack.size(); ++j)
      {
        cell::ptr c = graph[stack[j]];
        c->stop();
      }
  }

  int scheduler::execute(unsigned niter, unsigned nthread)
  {
    stop_running = false;
    compute_stack();
    notify_start();

    PyEval_InitThreads();
    assert(PyEval_ThreadsInitialized());

    //ECTO_START();
    recursive_mutex::scoped_lock lock(iface_mtx);
    {
      recursive_mutex::scoped_lock lock(running_mtx);
      if (running())
        BOOST_THROW_EXCEPTION(EctoException()
                              << diag_msg("threadpool scheduler already running"));
      running(true);
    }

    if (nthread == 0)
      nthread = boost::thread::hardware_concurrency();

    ECTO_LOG_DEBUG("%sstart execute_impl", "");
    int rv = execute_impl(niter, nthread);
    ECTO_LOG_DEBUG("%sdone execute_impl", "");

    running(false);
    notify_stop();

    return rv;
  }

  scheduler::exec::exec(scheduler& s_, unsigned niter_, unsigned nthread_)
    : s(s_), niter(niter_), nthread(nthread_)
  { }

  scheduler::exec::exec(const exec& rhs)
    : s(rhs.s)
    , niter(rhs.niter)
    , nthread(rhs.nthread)
  { }

  void scheduler::exec::operator()()
  {
    ECTO_START();
    s.running(true);
    s.execute_impl(niter, nthread);
    PyErr_CheckSignals();
    s.running(false);
    s.notify_stop();
    ECTO_FINISH();
  }

  void scheduler::execute_async(unsigned niter, unsigned nthread)
  {
    ECTO_START();
    recursive_mutex::scoped_lock lock(iface_mtx);
    if (running())
      BOOST_THROW_EXCEPTION(EctoException()
                            << diag_msg("threadpool scheduler already running"));
    running(true);
    stop_running = false;
    compute_stack();
    notify_start();

    if (nthread == 0)
      nthread = boost::thread::hardware_concurrency();

    exec e(*this, niter, nthread);

    boost::scoped_ptr<thread> tmp(new thread(e));

    tmp->swap(runthread);
  }

  void scheduler::compute_stack()
  {
    ECTO_START();
    if (!stack.empty()) //will be empty if this needs to be computed.
      return;
    //check this plasm for correctness.
    plasm_->check();
    plasm_->configure_all();
    boost::topological_sort(graph, std::back_inserter(stack));
    std::reverse(stack.begin(), stack.end());
  }


  int scheduler::invoke_process(graph_t::vertex_descriptor vd)
  {
    //    assert(false);
    //    ECTO_START();
    int rv;
    try {
      rv = ecto::schedulers::invoke_process(graph, vd);
    } catch (const boost::thread_interrupted& e) {
      std::cout << "Interrupted\n";
      return ecto::QUIT;
    }
    return rv;
  }

  void scheduler::stop()
  {
    ECTO_START();
    recursive_mutex::scoped_lock lock(iface_mtx);
    stop_running = true;
    stop_impl();
  }

  void scheduler::interrupt()
  {
    ECTO_START();
    recursive_mutex::scoped_lock lock(iface_mtx);
    interrupt_impl();
  }

  bool scheduler::running() const
  {
    recursive_mutex::scoped_lock lock(iface_mtx);
    recursive_mutex::scoped_lock running_lock(running_mtx);
    return running_value;
  }

  void scheduler::wait()
  {
    ECTO_START();
    recursive_mutex::scoped_lock lock(iface_mtx);
    wait_impl();
  }

  void
  scheduler::running(bool value)
  {
    boost::recursive_mutex::scoped_lock lock(running_mtx);
    running_value = value;
    running_cond.notify_all();
    ECTO_LOG_DEBUG("running=%u", value);
  }
}

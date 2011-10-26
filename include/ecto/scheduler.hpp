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
#pragma once

#include <ecto/log.hpp>
#include <ecto/forward.hpp>
#include <ecto/plasm.hpp>
#include <ecto/strand.hpp>
#include <ecto/profile.hpp>
#include <ecto/cell.hpp>
#include <ecto/atomic.hpp>

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/unordered_map.hpp>

#include <ecto/impl/graph_types.hpp>

namespace ecto {

  void verbose_run(boost::asio::io_service& s, std::string name);

  struct scheduler {

    explicit scheduler(plasm_ptr p);

    virtual ~scheduler();

    int execute(unsigned niter=0, unsigned nthread=0);
    void execute_async(unsigned niter=0, unsigned nthread=0);

    void stop();
    void interrupt();
    bool running() const;
    void wait();

    std::string stats();

  protected:

    virtual int execute_impl(unsigned niter, unsigned nthread, boost::asio::io_service& topserv) = 0;
    virtual void stop_impl() = 0;
    virtual void interrupt_impl() = 0;
    virtual void wait_impl() = 0;

    void running(bool);

    int invoke_process(ecto::graph::graph_t::vertex_descriptor vd);
    void compute_stack();

    plasm_ptr plasm;
    ecto::graph::graph_t& graph;

    std::vector<ecto::graph::graph_t::vertex_descriptor> stack;

    profile::graph_stats_type graphstats;

    boost::thread runthread;

    boost::asio::io_service top_serv;

  private:

    void notify_start();
    void notify_stop();

    struct exec {
      scheduler& s;
      unsigned niter, nthread;
      exec(const exec&);
      exec(scheduler& s_, unsigned niter_, unsigned nthread_);
      void operator()();
    };

    bool running_value;
    boost::condition_variable running_cond;
    mutable boost::recursive_mutex running_mtx;

    mutable boost::recursive_mutex iface_mtx;
  };

}

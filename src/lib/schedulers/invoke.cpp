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

#include <ecto/python.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

#include <ecto/log.hpp>
#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/cell.hpp>
#include <ecto/edge.hpp>

#include <ecto/impl/graph_types.hpp>
#include <ecto/impl/schedulers/access.hpp>
#include <ecto/plasm.hpp>

namespace ecto {

  using namespace ecto::graph;

  namespace schedulers {

    int
    invoke_process(graph_t& graph, graph_t::vertex_descriptor vd)
    {
      cell::ptr m = graph[vd];

      std::size_t tick = m->tick();

      ECTO_LOG_DEBUG(">> process %s tick %u", m->name() % tick);

      if (m->stop_requested()) {
        ECTO_LOG_DEBUG("%s Not processing because stop_requested", m->name());
        return ecto::QUIT;
      }
      graph_t::in_edge_iterator inbegin, inend;
      tie(inbegin, inend) = boost::in_edges(vd, graph);

      while (inbegin != inend)
        {
          edge_ptr e = graph[*inbegin];
          tendril& from = e->front();
          tendril& to = *(m->inputs[e->to_port()]);
          ECTO_LOG_DEBUG("Moving inputs to cell %s: tick=%u, from.tick=%u", m->name() % tick % from.tick);
          ECTO_ASSERT(tick == from.tick, "Internal scheduler error, graph has become somehow desynchronized.");
          to << from;
          // ECTO_ASSERT(to.tick == tick, "Graph has become somehow desynchronized");
          e->pop_front(); //todo Make this use a pool, instead of popping. To get rid of allocations.
          ++inbegin;
        }
      //verify that all inputs have been set.
      m->verify_inputs();

      int rval;
      try { rval = m->process(); } catch (...) { m->stop_requested(true); throw; }

      if(rval != ecto::OK) {
        ECTO_LOG_DEBUG("** process %s tick %u *BAILOUT*", m->name() % tick);
        return rval; //short circuit.
      }
      graph_t::out_edge_iterator outbegin, outend;
      tie(outbegin, outend) = boost::out_edges(vd, graph);
      while (outbegin != outend)
        {
          edge_ptr e = graph[*outbegin];
          tendril& from = *(m->outputs[e->from_port()]);
          from.tick = tick;
          // ECTO_LOG_DEBUG("%s Put output with tick %u", m->name() % from.tick);
          e->push_back(from);//copy everything... value, docs, user_defined, etc...
          ++outbegin;
        }
      m->inc_tick();
      // ECTO_LOG_DEBUG("Incrementing tick on %s to %u", m->name() % m->tick());
      ECTO_LOG_DEBUG("<< process %s tick %u", m->name() % tick);
      return rval;
    }
  }
}

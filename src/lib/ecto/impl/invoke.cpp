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

#include <ecto/cell.hpp>
#include <ecto/edge.hpp>
#include <ecto/impl/graph_types.hpp>
#include <ecto/log.hpp>
#include <ecto/plasm.hpp>
#include <ecto/python.hpp>
#include <ecto/tendril.hpp>
#include <ecto/vertex.hpp>

#include <boost/format.hpp>

#include <deque>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace ecto {
using namespace ecto::graph;

namespace schedulers {

void move_inputs(graph_t& graph, graph_t::vertex_descriptor vd)
{
  vertex_ptr v = graph[vd];
  cell::ptr m = v->cell();
  const std::string name = m->name();
#if defined(ECTO_LOGGING)
  const std::size_t tick = v->tick();
#endif

  graph_t::in_edge_iterator inbegin, inend;
  tie(inbegin, inend) = boost::in_edges(vd, graph);

  for (; inbegin != inend; ++inbegin) {
    edge_ptr e = graph[*inbegin];
    if (e->empty()) {
      // TODO (JTF): This can be true if a cell requests a DO_OVER, so how do
      // we validate inputs correctly?
      continue;
    }

    tendril& from = e->front();
    tendril& to = *(m->inputs[e->to_port()]);
    ECTO_LOG_DEBUG("Moving inputs to cell %s: tick=%u, from.tick=%u",
                   name % tick % e->tick());

    try {
      to << from;
    } catch(const ecto::except::EctoException& ex) {
      BOOST_THROW_EXCEPTION(except::CellException()
        << except::type(name_of(typeid(ex)))
        << except::what(ex.what())
        << except::cell_name(name)
        << except::when(boost::str(
             boost::format("Copying %s to %s")%e->from_port()%e->to_port())));
      throw;
    }
    e->pop_front(); // TODO: Make this use a pool instead of popping (to get rid of allocations).
  }
  // Verify that all inputs have been set.
  m->verify_inputs();
}

void move_outputs(graph_t& graph, graph_t::vertex_descriptor vd)
{
  vertex_ptr v = graph[vd];
  cell::ptr m = v->cell();
#if defined(ECTO_LOGGING)
  const std::string name = m->name();
#endif

  graph_t::out_edge_iterator outbegin, outend;
  tie(outbegin, outend) = boost::out_edges(vd, graph);

  while (outbegin != outend) {
    edge_ptr e = graph[*outbegin];
    assert(v->tick() == e->tick());
    tendril& from = *(m->outputs[e->from_port()]);
    ECTO_LOG_DEBUG("%s Put output with tick %u", name % e->tick());
    e->push_back(from);//copy everything... value, docs, user_defined, etc...
    e->inc_tick();
    ++outbegin;
  }

  v->inc_tick();
  ECTO_LOG_DEBUG("<< process %s tick %u", name % v->tick());
}

int invoke_process(graph_t& graph, graph_t::vertex_descriptor vd)
{
  vertex_ptr v = graph[vd];
  cell::ptr m = v->cell();
#if defined(ECTO_LOGGING)
  const std::string name = m->name();
  const std::size_t tick = v->tick();
#endif
  ECTO_LOG_DEBUG(">> process %s tick %u", name % tick);

  move_inputs(graph, vd);

  int rval = ecto::QUIT;
  { // BEGIN stats_collector scope.
    profile::stats_collector c(m->name(), v->stats());
    rval = m->process();
  } // END stats_collector scope.

  if (rval != ecto::OK) {
    ECTO_LOG_DEBUG("** process %s tick %u *BAILOUT*", name % tick);
    return rval; // short circuit.
  }

  move_outputs(graph, vd);

  return rval;
}

} // End of namespace schedulers.
} // End of namespace ecto.

/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <deque>

//this quiets a deprecated warning
#define BOOST_NO_HASH
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/foreach.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread.hpp>

#include <ecto/module.hpp>

namespace ecto {
  namespace graph {

    struct edge
    {
      edge(const std::string& fp, const std::string& tp) 
        : from_port(fp), to_port(tp)
      { }

      std::string from_port, to_port;
      typedef boost::shared_ptr<edge> ptr;
      typedef boost::shared_ptr<const edge> const_ptr;

      tendril& front() 
      { 
        boost::unique_lock<boost::mutex> lock(mtx);
        return deque.front();
      }
      void pop_front() 
      { 
        boost::unique_lock<boost::mutex> lock(mtx);
        deque.pop_front(); 
      }
      void push_back(const ecto::tendril& t) 
      {
        boost::unique_lock<boost::mutex> lock(mtx);
        deque.push_back(t);
      }
      std::size_t size() 
      {
        boost::unique_lock<boost::mutex> lock(mtx);
        return deque.size(); 
      }

    private:
      boost::mutex mtx;
      std::deque<ecto::tendril> deque;
    };


    // if the first argument is a sequence type (vecS, etc) then parallel edges are allowed
    typedef boost::adjacency_list<boost::vecS, // OutEdgeList...
                                  boost::vecS, // VertexList
                                  boost::bidirectionalS, // Directed
                                  module::ptr, // vertex property
                                  edge::ptr> // edge property
    graph_t;
  



    struct vertex_writer
    {
      graph_t* g;

      vertex_writer(graph_t* g_) : g(g_) { }

      void operator()(std::ostream& out, graph_t::vertex_descriptor vd)
      {
        out << "[label=\"" << (*g)[vd]->name() << "\"]";
      }
    };

    struct edge_writer
    {
      graph_t* g;

      edge_writer(graph_t* g_) :
        g(g_)
      { }

      void operator()(std::ostream& out, graph_t::edge_descriptor ed)
      {
        out << "[headlabel=\"" << (*g)[ed]->to_port << "\" taillabel=\"" << (*g)[ed]->from_port << "\"]";
      }
    };

    struct graph_writer
    {
      void operator()(std::ostream& out) const
      {
        out << "graph [rankdir=TB, ranksep=1]" << std::endl;
        out << "edge [labelfontsize=8]" << std::endl;
      }
    };

  }
}

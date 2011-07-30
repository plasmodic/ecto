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

#include <ecto/ecto.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/foreach.hpp>
#include <deque>

namespace ecto
{
  namespace bp = boost::python;

  struct Source
  {
    typedef boost::shared_ptr<Source> ptr;

    static void declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      out.declare<tendril::none>("out", "The delayed output");
    }

    void configure(tendrils& p, tendrils& i, tendrils& o)
    {
    }

    int process(tendrils& in, tendrils& out)
    {
      boost::mutex::scoped_lock lock(mtx);
      bp::object obj = queue.back().get<bp::object>();
      double d = bp::extract<double>(obj);
      std::cout << "val:" << d << "\n";
      out["out"] << queue.back();
      queue.pop_back();
      queue.push_front(*link->get<tendril::ptr>());
      std::cout << "q size = " << queue.size() << "\n";
      return ecto::OK;
    }

    void enqueue(tendril::ptr t)
    {
      boost::mutex::scoped_lock lock(mtx);
      queue.push_front(*t);
    }

    tendril::ptr link;
    std::deque<tendril> queue;
    boost::mutex mtx;
  };

  struct Sink
  {
    typedef boost::shared_ptr<Sink> ptr;

    static void declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      in.declare<tendril::none>("in", 
                                "Whatever goes in here appears in the output of"
                                "the associated source cell");
    }

    int process(tendrils& in, tendrils& out)
    {
      source->enqueue(in["in"]);
      return ecto::OK;
    }

    Source::ptr source;
  };

  struct Delay {
    typedef boost::shared_ptr<Delay> ptr;

    cell_<Source>::ptr source;
    cell_<Sink>::ptr sink;

    static void declare_params(tendrils& p)
    {
      p.declare<tendril::none>("default", "The default value").required(true);
      p.declare<unsigned>("qsize", "Size of queue", 1);
    }

    static void declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      
    }

    Delay() {
      source = create_cell<Source>();
      sink = create_cell<Sink>();
      sink->impl.reset(new Sink);
      source->impl.reset(new Source);
      sink->impl->source = source->impl;
      
      std::cout << "source=" << source.get() << "\n";
    }

    void configure(tendrils& p, tendrils& i, tendrils& o)
    {
      unsigned qsize = p["qsize"]->get<unsigned>();
      tendril::ptr def = p["default"];
      for (unsigned j=0; j<qsize; ++j)
        {
          source->impl->enqueue(def);
        }   
    }
  };


  namespace py {

    namespace {
      cell::ptr getsrc(cell_<Delay>& d)
      {
        if (! d.impl)
          d.impl.reset(new Delay);
        return d.impl->source;
      }
      cell::ptr getsink(cell_<Delay>& d)
      {
        if (! d.impl)
          d.impl.reset(new Delay);
        return d.impl->sink;
      }
    }
    void wrapDelay() {

      ecto::wrap<Delay>("Delay",
                        "Delay cell containing a source and a sink cell")
        .add_property("source", &getsrc)
        .add_property("sink", &getsink)
        ;
    };
  }

}

ECTO_CELL(ecto, ecto::Source, "Source", "Source cell of delay-pair");
ECTO_CELL(ecto, ecto::Sink,   "Sink",   "Sink cell of delay-pair");

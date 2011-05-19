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
#include <iostream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

namespace buster
{
struct FooPOD
{
  int x;
  float y;
};

struct FooPODModule
{
  static void Initialize(ecto::tendrils& p)
  {
    p.declare<std::string> ("str", "I print this:", "Hello World");
  }

  void configure(const ecto::tendrils& parameters, ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    inputs.declare<FooPOD> ("foo", "A string to print");
    outputs.declare<FooPOD> ("foo", "A string to print");
  }

  void process(const ecto::tendrils& parameters, const ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    std::cout << inputs.get<FooPOD> ("foo").x << std::endl;
    outputs.get<FooPOD> ("foo").y = 3.14;
  }
};
struct Printer
{
  static void Initialize(ecto::tendrils& p)
  {
    p.declare<std::string> ("str", "I print this:", "Hello World");
  }

  void configure(const ecto::tendrils& parameters, ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    inputs.declare<std::string> ("str", "A string to print", "hello");
  }

  void process(const ecto::tendrils& parameters, const ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    std::cout << inputs.get<std::string> ("str") << std::endl;
  }
};

struct Generate
{
  int step_;

  static void Initialize(ecto::tendrils& p)
  {
    p.declare<double> ("step", "The step with which i generate integers.", 2);
    p.declare<double> ("start", "My starting value", 0);
  }

  void configure(const ecto::tendrils& parameters, ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    step_ = parameters.get<double> ("step");
    outputs.declare<double> ("out", "output",
        parameters.get<double> ("start") - step_);
  }

  void process(const ecto::tendrils& parameters, const ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    outputs.get<double> ("out") += step_;
  }
};

struct Multiply
{
  double factor_;

  static void Initialize(ecto::tendrils& p)
  {
    p.declare<double> ("factor", "A factor to multiply by.", 3.14);
  }

  void configure(const ecto::tendrils& parameters, ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    factor_ = parameters.get<double> ("factor");
    inputs.declare<double> ("in", "multly in by factor");
    outputs.declare<double> ("out", "the result of in * factor");
  }
  void process(const ecto::tendrils& parameters, const ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    outputs.get<double> ("out") = inputs.get<double> ("in") * factor_;
  }
};
struct SharedPass
{

  typedef ecto::Handle<boost::shared_ptr<int> > handle_t;
  typedef ecto::ConstHandle<boost::shared_ptr<int> > handle_const_t;

  static void Initialize(ecto::tendrils& p)
  {
    p.declare<int> ("x", "Default value", -1);
  }

  void configure(const ecto::tendrils& parameters, ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    input = inputs.declare<boost::shared_ptr<int> > ("input", "a pass through",
        boost::shared_ptr<int>(new int(parameters.get<int> ("x"))));
    output = outputs.declare<boost::shared_ptr<int> > ("output",
        "a pass through", *input);
    value = outputs.declare<int> ("value", "value", -1);
  }
  void process(const ecto::tendrils& parameters, const ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    *output = *input;
    //std::cout << *output << std::endl;
    *value = **output;
  }
  handle_t input;
  handle_t output;
  ecto::Handle<int> value;
};
struct Scatter
{
  static void Initialize(ecto::tendrils& p)
  {
    p.declare<int> ("n", "Number to scatter...", 2);
    p.declare<int> ("x", "The value to scatter...", 13);
  }

  void configure(const ecto::tendrils& parameters, ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    n_ = parameters.get<int> ("n");
    x_ = parameters.get<int> ("x");
    for (int i = 0; i < n_; i++)
    {
      outputs.declare<int> (str(boost::format("out_%04d") % i),
          str(boost::format("The %dth scatter") % i));
    }
  }
  void process(const ecto::tendrils& parameters, const ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    for (int i = 0; i < n_; i++)
    {
      outputs.get<int> (str(boost::format("out_%04d") % i)) = x_;
    }
  }
  int n_, x_;
};

template<typename ValueT>
struct Gather
{
  typedef ValueT value_type;

  static void Initialize(ecto::tendrils& p)
  {
    p.declare<int> ("n", "N to gather", 2);
  }

  void configure(const ecto::tendrils& parameters, ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    n_ = parameters.get<int> ("n");
    for (int ii = 0; ii < n_; ii++)
    {
      inputs.declare<value_type> (str(boost::format("in_%04d") % ii),
          "An " + ecto::name_of<value_type>() + "input.");
    }
    outputs.declare<value_type> ("out", "The sum of all inputs.");
  }

  void process(const ecto::tendrils& parameters, const ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    //SHOW();
    value_type& out = outputs.get<value_type> ("out");
    out = 0;
    typedef std::pair<std::string, ecto::tendril> pp;
    BOOST_FOREACH(const pp& in,inputs)
          {
            out += in.second.get<value_type> ();
          }
  }
  int n_;
};

boost::shared_ptr<ecto::tendril> makePodTendril()
{
  boost::shared_ptr<ecto::tendril> p;
  ecto::tendril* t = new ecto::tendril(0, "doc");
  p.reset(t);
  return p;
}

}

BOOST_PYTHON_MODULE(buster)
{
  using namespace buster;
  ecto::wrap<Printer>("Printer", "A printer...");
  ecto::wrap<Generate>("Generate", "A generator module.");
  ecto::wrap<SharedPass>("SharedPass", "A shared pointer pass through");
  ecto::wrap<Multiply>("Multiply", "Multiply an input with a constant");
  ecto::wrap<Scatter>("Scatter", "Scatter a value...");
  ecto::wrap<Gather<int> >("Gather", "Gather a scattered value...");
  ecto::wrap<Gather<double> >("Gather_double", "Gather a scattered value...");
  boost::python::def("make_pod_tendril", buster::makePodTendril);
}

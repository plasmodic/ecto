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
#include <boost/asio.hpp>
#include <boost/exception/all.hpp>

using ecto::tendrils;
namespace ecto_test
{
  struct FooPOD
  {
    int x;
    float y;
  };

  struct EvilNoPython
  {
    std::string Woz;
  };

  struct FooPODModule
  {
    static void declare_params(tendrils& parameters)
    {
      parameters.declare<std::string> ("str", "I print this:", "Hello World");
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<FooPOD> ("foo", "A string to print");
      outputs.declare<FooPOD> ("foo", "A string to print");
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      std::cout << inputs.get<FooPOD> ("foo").x << std::endl;
      outputs.get<FooPOD> ("foo").y = 3.14;
      return ecto::OK;
    }

  };

  struct NoPythonBindings
  {
    static void declare_params(tendrils& parameters)
    {
      parameters.declare<EvilNoPython> ("Woz", "A Woz is a Woz when a Woz was Woz");
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.declare<EvilNoPython> ("Strasz", "A Strasz is a Strasz when a Strasz saw a Strasz");
    }
  };

  struct DontAllocateMe
  {
    static void declare_params(tendrils& parameters)
    {
      parameters.declare<std::string> ("str");
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.declare<std::string> ("str");
    }

    DontAllocateMe()
    {
      std::cout << "Nuh-uh... I'm gonna throw now." << std::endl;
      throw std::logic_error("I shouldn't be allocated");
    }
  };

  struct DontCallMeFromTwoThreads
  {
    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<double> ("in");
      outputs.declare<double> ("out");
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      boost::asio::io_service s;
      boost::asio::deadline_timer dt(s);

      // try to rock
      if (mtx.try_lock())
        {
          // we got the rock... i.e. we are rocking
          std::cout << this << " got the lock." << std::endl;
          // wait a bit so's we can be sure there will be collisions
          dt.expires_from_now(boost::posix_time::milliseconds(250));
          dt.wait();

          double value = inputs.get<double> ("in");
          std::cout << "nonconcurrent node @ " << this << " moving " << value << std::endl;
          // do yer thing
          outputs.get<double> ("out") = value;

          // unrock
          std::cout << this << " done with the lock." << std::endl;
          mtx.unlock();
        }
      else
        {
          std::cout << this << " did NOT get the lock, I'm going to throw about this." << std::endl;
          BOOST_THROW_EXCEPTION(std::runtime_error("AAAAGH NO LOCK HEEEEEELP"));
          assert(false && "we should NOT be here");
          // throw std::logic_error("Didn't get the lock... this means we were called from two threads.  Baaad.");
        }
      return ecto::OK;

    }
    static boost::mutex mtx;
  };
  boost::mutex DontCallMeFromTwoThreads::mtx;

  struct Printer
  {
    static void declare_params(tendrils& parameters)
    {
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<double> ("in", "what to print");
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      std::cout << "***** " << inputs.get<double> ("in") << " ***** " << this << std::endl;
      return ecto::OK;
    }
  };

  template<typename T>
  struct Generate
  {
    T step_;

    static void declare_params(tendrils& parameters)
    {
      parameters.declare<T> ("step", "The step with which i generate integers.", 2);
      parameters.declare<T> ("start", "My starting value", 0);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.declare<T> ("out", "output", parameters.get<T> ("start") - parameters.get<T> ("step"));
    }

    void configure(tendrils& parameters, tendrils& inputs, tendrils& outputs)
    {
      step_ = parameters.get<T> ("step");
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.get<T> ("out") += step_;
      return 0;
    }
  };

  struct Quitter
  {
    static void declare_params(tendrils& params)
    {
      params.declare<std::string> ("str", "The default string to print", "EXIT");
    }

    static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<std::string> ("str", "The input string to listen to.", "");
    }

    void configure(tendrils& parms, tendrils& inputs, tendrils& outputs)
    {
      stop_word_ = parms.get<std::string> ("str");
    }

    int process(const tendrils& in, tendrils& /*out*/)
    {
      if (in.get<std::string> ("str") == stop_word_)
        return ecto::QUIT;
      return ecto::OK;
    }
    std::string stop_word_;
  };

  struct Multiply
  {
    double factor_;

    static void declare_params(ecto::tendrils& p)
    {
      p.declare<double> ("factor", "A factor to multiply by.", 3.14);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<double> ("in", "multly in by factor");
      outputs.declare<double> ("out", "the result of in * factor");
    }

    void configure(tendrils& parms, tendrils& inputs, tendrils& outputs)
    {
      factor_ = parms.get<double> ("factor");
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.get<double> ("out") = inputs.get<double> ("in") * factor_;
      return ecto::OK;
    }
  };

  struct Increment
  {
    double amount_;
    unsigned delay_ms_;
    static void declare_params(ecto::tendrils& p)
    {
      p.declare<double> ("amount", "Amount to increment by.", 1.0);
      p.declare<unsigned> ("delay", "How long it takes to increment in milliseconds", 0);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<double> ("in", "input");
      outputs.declare<double> ("out", "output");
    }

    void configure(tendrils& parms, tendrils& inputs, tendrils& outputs)
    {
      amount_ = parms.get<double> ("amount");
      delay_ms_ = parms.get<unsigned> ("delay");
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      if (delay_ms_ > 0)
        {
          boost::asio::io_service s;
          boost::asio::deadline_timer dt(s);
          dt.expires_from_now(boost::posix_time::milliseconds(delay_ms_));
          dt.wait();
        }
      double in = inputs.get<double> ("in");
      double result = in + amount_;
      // std::cout << this << " incrementer: " << in << " ==> " << result << std::endl;
      outputs.get<double> ("out") = result;
      return ecto::OK;
    }
  };

  struct Add
  {
    static void declare_params(ecto::tendrils& p)
    {
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<double> ("left", "left input");
      inputs.declare<double> ("right", "right input");
      outputs.declare<double> ("out", "output");
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.get<double> ("out") = inputs.get<double> ("left") + inputs.get<double> ("right");
      return ecto::OK;
    }
  };

  struct ParameterWatcher
  {
    double value_;

    static void declare_params(ecto::tendrils& p)
    {
      p.declare<double> ("value", "I use this value", 1.0);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<double> ("input", "input");
      outputs.declare<double> ("output", "output");
      outputs.declare<double> ("value", "the parameter.");

    }

    void onvalue_change(double v)
    {
      SHOW();
      std::cout << "old value: " << value_ << std::endl;
      std::cout << "new value: " << v << std::endl;
      value_ = v;
    }

    void configure(tendrils& parms, tendrils& inputs, tendrils& outputs)
    {
      parms.at("value")->set_callback<double> (boost::bind(&ParameterWatcher::onvalue_change, this, _1));
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.get<double> ("output") = inputs.get<double> ("input") * value_;
      outputs.get<double> ("value") = value_;
      return ecto::OK;
    }
  };

  struct HandleHolder
  {
    ecto::spore<double> value_, input_, output_, param_val_;

    static void declare_params(ecto::tendrils& p)
    {
      p.declare<double> ("value", "I use this value", 1.0);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<double> ("input", "input");
      outputs.declare<double> ("output", "output");
      outputs.declare<double> ("value", "the parameter.");

    }

    void onvalue_change(double v)
    {
      SHOW();
      std::cout << "my value: " << *value_ << std::endl;
      std::cout << "new value: " << v << std::endl;
      if(*value_ != v) throw std::runtime_error("The new value should equal the old value!");
    }

    void configure(tendrils& parms, tendrils& inputs, tendrils& outputs)
    {
      value_ = parms.at("value");
      value_.set_callback(boost::bind(&HandleHolder::onvalue_change, this, _1));
      output_ = outputs.at("output");
      input_ = inputs.at("input");
      param_val_ = outputs.at("value");
    }

    int process(const ecto::tendrils&, ecto::tendrils&)
    {
      *output_ = (*input_) * (*value_);
      *param_val_ = *value_;
      return ecto::OK;
    }
  };

  struct SharedPass
  {
    typedef boost::shared_ptr<int> ptr_t;

    static void declare_params(ecto::tendrils& p)
    {
      p.declare<int> ("x", "Default value", -1);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<ptr_t> ("input", "a pass through", ptr_t(new int(parameters.get<int> ("x"))));

      outputs.declare<ptr_t> ("output", "a pass through", ptr_t(new int(-1)));

      outputs.declare<int> ("value", "value", -1);
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.get<ptr_t> ("output") = inputs.get<ptr_t> ("input");
      outputs.get<int> ("value") = *outputs.get<ptr_t> ("output");
      return ecto::OK;
    }
  };

  struct Scatter
  {
    static void declare_params(ecto::tendrils& p)
    {
      p.declare<int> ("n", "Number to scatter...", 2);
      p.declare<int> ("x", "The value to scatter...", 13);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      int n = parameters.get<int> ("n");
      for (int i = 0; i < n; i++)
        {
          outputs.declare<int> (str(boost::format("out_%04d") % i), str(boost::format("The %dth scatter") % i));
        }
    }

    void configure(tendrils& parameters, tendrils& inputs, tendrils& outputs)
    {
      n_ = parameters.get<int> ("n");
      x_ = parameters.get<int> ("x");
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      for (int i = 0; i < n_; i++)
        {
          outputs.get<int> (str(boost::format("out_%04d") % i)) = x_;
        }
      return ecto::OK;
    }

    int n_, x_;
  };

  template<typename ValueT>
  struct Gather
  {
    typedef ValueT value_type;

    static void declare_params(ecto::tendrils& p)
    {
      p.declare<int> ("n", "N to gather", 2);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      int n = parameters.get<int> ("n");
      for (int ii = 0; ii < n; ii++)
        {
          inputs.declare<value_type> (str(boost::format("in_%04d") % ii),
                                      "An " + ecto::name_of<value_type>() + "input.");
        }
      outputs.declare<value_type> ("out", "The sum of all inputs.");
    }

    void configure(tendrils& parameters, tendrils& inputs, tendrils& outputs)
    {
      n_ = parameters.get<int> ("n");
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      //SHOW();
      value_type& out = outputs.get<value_type> ("out");
      out = 0;
      typedef std::pair<std::string, ecto::tendril::ptr> pp;
      BOOST_FOREACH(const pp& in,inputs)
              {
                out += in.second->get<value_type> ();
              }
      return ecto::OK;
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

BOOST_PYTHON_MODULE(ecto_test)
{
  using namespace ecto_test;
  ecto::wrap<Printer>("Printer", "A printer of doubles...");
  ecto::wrap<Generate<double> >("Generate", "A generator module.");
  ecto::wrap<SharedPass>("SharedPass", "A shared pointer pass through");
  ecto::wrap<Multiply>("Multiply", "Multiply an input with a constant");
  ecto::wrap<Add>("Add", "Add two numbers");
  ecto::wrap<Increment>("Increment", "Increment input by some amount");
  ecto::wrap<Scatter>("Scatter", "Scatter a value...");
  ecto::wrap<Gather<int> >("Gather", "Gather a scattered value...");
  ecto::wrap<Gather<double> >("Gather_double", "Gather a scattered value...");
  ecto::wrap<Quitter>("Quitter", "Will quit the graph on an appropriate input.");
  ecto::wrap<HandleHolder>("HandleHolder","Holds on to handles...");
  ecto::wrap<DontAllocateMe>("DontAllocateMe", "Don't allocate me, feel free to inspect.");
  ecto::wrap<DontCallMeFromTwoThreads>("DontCallMeFromTwoThreads",
                                       "Throws if process called concurrently from two threads.");
  ecto::wrap<NoPythonBindings>("NoPythonBindings", "This uses something that is bound to python!");
  ecto::wrap<ParameterWatcher>("ParameterWatcher", "Uses parameter change callbacks.");
  boost::python::def("make_pod_tendril", ecto_test::makePodTendril);
}

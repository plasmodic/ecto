#include <ecto/ecto.hpp>
#include <iostream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

namespace buster
{
  struct Printer : ecto::module
  {
    static void Params(ecto::tendrils& p)
    {
      p.declare<std::string> ("str","I print this:", "Hello World");
    }

    void Config()
    {
      inputs.declare<std::string>("str","A string to print","hello");
    }

    void Process()
    {
      std::cout << inputs.get<std::string>("str") << std::endl;
    }
  };

  struct Generate : ecto::module
  {
    int step_;

    static void Params(ecto::tendrils& p)
    {
      p["step"].set<double> ("The step with which i generate integers.", 2);
      p["start"].set<double> ("My starting value", 0);
    }

    void Config()
    {
      step_ = params.get<double> ("step");
      outputs.declare<double>("out", "output", params.get<double> ("start") - step_);
    }

    void Process()
    {
      outputs.get<double> ("out") += step_;
    }
  };

  struct Multiply : ecto::module
  {
    double factor_;

    static void Params(ecto::tendrils& p)
    {
      p["factor"].set<double> ("A factor to multiply by.", 3.14);
    }

    void Config()
    {
      factor_ = params.get<double> ("factor");
      inputs.declare<double> ("in", "multly in by factor");
      outputs.declare<double> ("out", "the result of in * factor");
    }
    void Process()
    {
      outputs.get<double> ("out") = inputs.get<double> ("in") * factor_;
    }
  };

  struct Scatter : ecto::module
  {
    static void Params(ecto::tendrils& p)
    {
      p["n"].set<int> ("Number to scatter...", 2);
      p["x"].set<int> ("The value to scatter...", 13);
    }

    void Config()
    {
      n_ = params.get<int> ("n");
      x_ = params.get<int> ("x");
      for (int i = 0; i < n_; i++)
	{
	  outputs.declare<int> (str(boost::format("out_%04d") % i), "The ith scater");
	}
    }
    void Process()
    {
      for (int i = 0; i < n_; i++)
      {
      outputs.get<int> (str(boost::format("out_%04d") % i)) = x_;
      }
    }
    int n_, x_;
  };
using namespace ecto;
 template<typename ValueT>
  struct Gather : public ecto::module
  {
    typedef ValueT value_type;

    static void Params(ecto::tendrils& p)
    {
      p.declare<int> ("n", "N to gather", 2);
    }

    void Config()
    {
      n_ = params.get<int>("n");
      for (int ii = 0; ii < n_; ii++)
      {
        inputs.declare<value_type>(str(boost::format("in_%04d") % ii), "An " + ecto::name_of<value_type>() + "input.");
      }
      outputs.declare<value_type>("out", "The sum of all inputs.");
    }

    void Process()
    {
      //SHOW();
      value_type& out = outputs.get<value_type>("out");
      out = 0;
      typedef std::pair<std::string, ecto::tendril> pp;
      BOOST_FOREACH(const pp& in,inputs)
      {
        out += in.second.get<value_type> ();
      }
    }
    int n_;
  };

}

BOOST_PYTHON_MODULE(buster)
{
  using namespace buster;
  ecto::wrap<Printer>("Printer", "A printer...");
  ecto::wrap<Generate>("Generate", "A generator module.");
  ecto::wrap<Multiply>("Multiply", "Multiply an input with a constant");
  ecto::wrap<Scatter>("Scatter", "Scatter a value...");
  ecto::wrap<Gather<int> >("Gather", "Gather a scattered value...");
  ecto::wrap<Gather<double> >("Gather_double", "Gather a scattered value...");
}

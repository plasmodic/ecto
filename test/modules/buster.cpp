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
      i().declare<std::string>("str","A string to print","hello");
    }

    void Process()
    {
      std::cout << i().get<std::string>("str") << std::endl;
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
      step_ = p().get<double> ("step");
      o().declare<double>("out", "output", p().get<double> ("start") - step_);
    }

    void Process()
    {
      o().get<double> ("out") += step_;
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
      factor_ = p().get<double> ("factor");
      i().declare<double> ("in", "multly in by factor");
      o().declare<double> ("out", "the result of in * factor");
    }
    void Process()
    {
      o().get<double> ("out") = i().get<double> ("in") * factor_;
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
      n_ = p().get<int> ("n");
      x_ = p().get<int> ("x");
      for (int i = 0; i < n_; i++)
	{
	  o().declare<int> (str(boost::format("out_%04d") % i), "The ith scater");
	}
    }
    void Process()
    {
      SHOW();
      for (int i = 0; i < n_; i++)
	{
        o().get<int> (str(boost::format("out_%04d") % i)) = x_;
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
      n_ = p().template get<int>("n");
      for (int ii = 0; ii < n_; ii++)
      {
        i().template declare<value_type>(str(boost::format("in_%04d") % ii), "An " + ecto::name_of<value_type>() + "input.");
      }
      o().template declare<value_type>("out", "The sum of all inputs.");
    }

    void Process()
    {
      //SHOW();
      value_type& out = o().template get<value_type>("out");
      out = 0;
      typedef std::pair<std::string, ecto::tendril> pp;
      BOOST_FOREACH(const pp& in,i())
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

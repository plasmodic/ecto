#include <ecto/ecto.hpp>
#include <iostream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

//disable show in here
#define DISABLE_SHOW 1
#if DISABLE_SHOW
#ifdef SHOW
#undef SHOW
#define SHOW() do{}while(false)
#endif
#endif

namespace buster
{

  struct Printer : ecto::module
  {

    static void Params(ecto::tendrils& p)
    {
      p["str"].set<std::string> ("I print this:", "Hello World");
    }

    void Config()
    {
      setIn<std::string>("str","A string to print","hello");
    }

    void Process()
    {
      std::cout << getIn<std::string>("str") << std::endl;
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
      step_ = getParam<double> ("step");
      setOut<double> ("out", "output", getParam<double> ("start") - step_);
    }

    void Process()
    {
      getOut<double> ("out") += step_;
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
      factor_ = getParam<double> ("factor");
      setIn<double> ("in", "multly in by factor");
      setOut<double> ("out", "the result of in * factor");
    }
    void Process()
    {
      getOut<double> ("out") = getIn<double> ("in") * factor_;
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
      n_ = getParam<int> ("n");
      x_ = getParam<int> ("x");
      for (int i = 0; i < n_; i++)
	{
	  setOut<int> (str(boost::format("out_%04d") % i), "The ith scater");
	}
    }
    void Process()
    {
      SHOW();
      for (int i = 0; i < n_; i++)
	{
	  getOut<int> (str(boost::format("out_%04d") % i)) = x_;
	}
    }
    int n_, x_;
  };

  struct Gather : ecto::module
  {
    static void Params(ecto::tendrils& p)
    {
      p["n"].set<int> ("N to gather", 2);
    }

    void Config()
    {
      n_ = getParam<int> ("n");
      for (int i = 0; i < n_; i++)
	{
	  setIn<int> (str(boost::format("in_%04d") % i), "An integer input.");
	}
      setOut<int> ("out", "The sum of all inputs.");
    }

    void Process()
    {
      SHOW();
      int& out = getOut<int> ("out");
      out = 0;
      typedef std::pair<std::string, ecto::tendril> pp;
      BOOST_FOREACH(const pp& in,inputs)
	{
	  out += in.second.get<int> ();
	}
    }
    int n_;
  };

}

ECTO_MODULE(buster)
{
  using namespace buster;
  ecto::wrap<Printer>("Printer", "A printer...");
  ecto::wrap<Generate>("Generate", "A generator module.");
  ecto::wrap<Multiply>("Multiply", "Multiply an input with a constant");
  ecto::wrap<Scatter>("Scatter", "Scatter a value...");
  ecto::wrap<Gather>("Gather", "Gather a scattered value...");
}

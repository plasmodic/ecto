#include <ecto/ecto.hpp>
#include <iostream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

struct Generate : ecto::module
{
  int step_;

  static void Params(connections_t& p)
  {
    p["step"].set<int> ("The step with which i generate integers.", 2);
    p["start"].set<int> ("My starting value", 0);
  }

  void Config()
  {
    step_ = getParam<int> ("step");
    setOut<int> ("out", "output", getParam<int> ("start"));
  }

  void Process()
  {
    getOut<int> ("out") += step_;
  }
};

struct Multiply : ecto::module
{
  int factor_;

  static void Params(connections_t& p)
  {
    p["factor"].set<float> ("A factor to multiply by.", 3.14);
  }

  void Config()
  {
    factor_ = getParam<float>("factor");
    setIn<int> ("in", "multly in by factor");
    setOut<int> ("out", "the result of in * factor");
  }
  void Process()
  {
    getOut<int> ("out") = getIn<int> ("in") * factor_;
  }
};

struct Scatter : ecto::module
{
  static void Params(connections_t& p)
  {
    p["n"].set<int> ("Number to scatter...", 2);
    p["x"].set<int> ("The value to scatter...", 13);
  }

  void Config()
  {
    n_ = getParam<int> ("n");
    x_ = getParam<int> ("x");
    setOut<std::vector<int> > ("out", str(boost::format("A vector, length n=%d, value=%d") % n_ % x_));
  }
  void Process()
  {
    SHOW();
    std::vector<int>& out = getOut<std::vector<int> > ("out");
    out = std::vector<int>(n_, x_);
  }
  int n_, x_;
};

struct Gather : ecto::module
{
  static void Params(connections_t& p)
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

struct Indexer : ecto::module
{
  static void Params(connections_t& p)
  {
    p["n"].set<int> ("N the number to index.", 0);
  }

  void Config()
  {
    n_ = getParam<int> ("n");
    setIn<std::vector<int> > ("in", "the vector to index", std::vector<int>(n_));
    setOut<int> ("out", str(boost::format("the output at in[%d]") % n_), 0);
  }
  void Process()
  {
    SHOW();
    const std::vector<int>& in = getIn<std::vector<int> > ("in");
    int & out = getOut<int> ("out");
    if (in.size() > size_t(n_))
      out = in[n_];
    else
      std::cerr << "wrong size! idx= " << n_ << " length of vector " << in.size() << std::endl;
  }
  int n_;
};

ECTO_MODULE(sample)
{
  ecto::wrap<Generate>("Generate");
  ecto::wrap<Multiply>("Multiply");
  ecto::wrap<Scatter>("Scatter");
  ecto::wrap<Indexer>("Indexer");
  ecto::wrap<Gather>("Gather");
}

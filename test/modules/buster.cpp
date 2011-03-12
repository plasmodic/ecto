#include <ecto/ecto.hpp>
#include <ecto/util.hpp>
#include <ecto/ecto_wrap.hpp>
#include <iostream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
struct OurModule : ecto::module
{
  OurModule()
  {
    inputs["in"] = ecto::connection::make<int>();
    outputs["out1"] = ecto::connection::make<float>();
    outputs["out2"] = ecto::connection::make<bool>();
  }

  void Config()
  {
  }
  void Process()
  {
  }

};

struct Generate : ecto::module
{
  int step_, start_;

  void Config(int start, int step)
  {
    //SHOW();
    start_ = start;
    step_ = step;
    setOut<int> ("out", "output", 0);
    setOut<float> ("out02", "What a blast", 0);
  }

  void Process()
  {
    //SHOW();
    int& o = getOut<int> ("out");
    o = start_ + step_;
    start_ += step_;
  }
};

struct Multiply : ecto::module
{
  int factor_;

  void Config(int factor)
  {
    //SHOW();
    factor_ = factor;
    setIn<int> ("in", "multly in by factor");
    setIn<float> ("fin", "float input");
    setOut<int> ("out", "the result of in * factor");
  }

  void Process()
  {
    //SHOW();
    const int& i = getIn<int> ("in");
    int& o = getOut<int> ("out");
    o = i * factor_;
  }
};

std::ostream& operator<<(std::ostream& out, const std::vector<int>& t)
{
  out << "[";
  BOOST_FOREACH(int x, t)
      out << x <<" ";
  return out<<"]";
}

struct Scatter : ecto::module
{
  void Config(int x, int n)
  {
    n_ = n;
    x_ = x;
    setOut<std::vector<int> >("out",str(boost::format("A vector, length n=%d, value=%d")%n_ %x_));
  }
  void Process()
  {
    std::vector<int>& out = getOut<std::vector<int> >("out");
    out = std::vector<int>(n_,x_);
  }
  int n_,x_;
};

struct Gather : ecto::module
{
  void Config(int n)
  {
    n_ = n;
    for(int i = 0; i < n_; i++)
    {
      setIn<int >(str(boost::format("in_%04d")%i),"An integer input.");
    }
    setOut<int> ("out", "The sum of all inputs.");
  }
  void Process()
  {
    int& out = getOut<int>("out");
    out = 0;
    typedef std::pair<std::string,ecto::connection> pp;
    BOOST_FOREACH(const pp& in,inputs)
    {
      out += in.second.get<int>();
    }
  }
  int n_;
};

struct Indexer : ecto::module
{
  void Config(int n)
  {
    n_ = n;
    setIn<std::vector<int> >("in", "the vector to index",std::vector<int>(n_));
    setOut<int >("out",str(boost::format("the output at in[%d]")%n_),0);
  }
  void Process()
  {
    const std::vector<int>& in = getIn<std::vector<int> >("in");
    int & out = getOut<int>("out");
    if(in.size() > size_t( n_))
    out = in[n_];
    else
      std::cerr << "wrong size! idx= " << n_ << " length of vector " << in.size() << std::endl;
  }
  int n_;
};
ECTO_MODULE(buster)
{
  ecto::wrap<OurModule>("OurModule");
  ecto::wrap<Generate>("Generate");
  ecto::wrap<Multiply>("Multiply");
  ecto::wrap<Scatter>("Scatter");
  ecto::wrap<Indexer>("Indexer");
  ecto::wrap<Gather>("Gather");
}

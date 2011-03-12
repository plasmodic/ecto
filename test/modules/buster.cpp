#include <ecto/ecto.hpp>
#include <ecto/util.hpp>
#include <ecto/ecto_wrap.hpp>
#include <iostream>

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
    SHOW();
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
    SHOW();
    factor_ = factor;
    setIn<int> ("in", "multly in by factor");
    setIn<float> ("fin", "float input");
    setOut<int> ("out", "the result of in * factor");
  }

  void Process()
  {
    SHOW();
    const int& i = getIn<int> ("in");
    int& o = getOut<int> ("out");
    o = i * factor_;
  }
};

ECTO_MODULE(buster)
{
  ecto::wrap<OurModule>("OurModule");
  ecto::wrap<Generate>("Generate");
  ecto::wrap<Multiply>("Multiply");
}

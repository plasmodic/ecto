#include <ecto/ecto.hpp>
#include <iostream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

#include <vector>
#include <numeric>

namespace ecto_push_ups
{
size_t big_data()
{
  std::vector<int> data;
  data.resize(10e4);
  for (size_t i = 0; i < 10e4; i++)
    data[i] = std::rand();
  return std::accumulate(data.begin(), data.end(), 0);
}
int add2(int x, int y)
{
  return x + y;
}

struct Add2 
{
  static void declare_io(const ecto::tendrils& /*parameters*/, ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    SHOW();
    outputs.declare<int> ("out", "x+y");
    inputs.declare<int> ("x");
    inputs.declare<int> ("y");
  }
  ecto::ReturnCode process(const ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    //SHOW();
    outputs.get<int> ("out") = add2(std::rand(), std::rand());
    return ecto::eOK;
  }

};

struct BigData 
{
  static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    SHOW();
    outputs.declare<int> ("out", "sum of random array");
  }
  ecto::ReturnCode process(const ecto::tendrils& inputs,
      ecto::tendrils& outputs)
  {
    outputs.get<int> ("out") = big_data();
    return ecto::eOK;
  }
};
}

BOOST_PYTHON_MODULE(push_ups)
{
  using namespace ecto_push_ups;
  ecto::wrap<Add2>("Add2", "An adder.");
  ecto::wrap<BigData>("BigData", "BigData...");
}

#include <gtest/gtest.h>
#include <ecto/ecto.hpp>

#define STRINGDIDLY(A) std::string(#A)

using namespace ecto;
struct Module1
{
  static void
  declare_io(const tendrils& p, tendrils& in, tendrils& out)
  {
    out.declare<double> ("d");
  }
};

struct Module2
{
  static void
  declare_io(const tendrils& p, tendrils& in, tendrils& out)
  {
    in.declare<double> ("d");
  }
};

TEST(Plasm, Viz)
{
  ecto::plasm p;
  ecto::cell::ptr m1 = ecto::create_cell<Module1>(), m2 = ecto::create_cell<Module2>();
  p.connect(m1,"d",m2,"d");
  std::cout << p.viz() << std::endl;
}

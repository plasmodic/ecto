#include <gtest/gtest.h>
#include <ecto/ecto.hpp>
#include <ecto/plasm.hpp>

using namespace ecto;
namespace {
  struct A
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<double> ("d","Doc",5.0);
      p.declare<std::string> ("x","Doc a string", "Hello");
    }
  };

}
TEST(Clone, Clone1)
{
  ecto::cell::ptr m1 = ecto::create_cell<A>();
  m1->parameters["x"] << std::string("Hello");
  m1->parameters["d"] << 7.0;

  ecto::cell::ptr mm1 = m1->clone();
  EXPECT_EQ(mm1->parameters.get<double>("d"),7.0);
  m1->parameters["d"] << 9.0;
  EXPECT_EQ(mm1->parameters.get<double>("d"),7.0);
}

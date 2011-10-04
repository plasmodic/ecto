#include <gtest/gtest.h>
#include <ecto/ecto.hpp>
#include <ecto/plasm.hpp>

using namespace ecto;
namespace {
  struct Add
  {
    static void declare_io(const ecto::tendrils& p, ecto::tendrils& i, ecto::tendrils& o)
    {
      i.declare(&Add::left_,"left");
      i.declare(&Add::right_,"right");
      o.declare(&Add::out_,"out");
    }
    int process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      *out_ = *left_ + *right_;
      return ecto::OK;
    }
    ecto::spore<double> out_, left_, right_;
  };
}
TEST(Static, DoesItWork)
{
  ecto::cell::ptr a = ecto::inspect_cell<Add>();
  a->inputs["left"] << 2.0;
  a->inputs["right"] << 5.0;
  a->process();
  EXPECT_EQ(a->outputs.get<double>("out"),7.0);
}

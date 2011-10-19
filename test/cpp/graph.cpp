#include <gtest/gtest.h>
#include <ecto/ecto.hpp>
#include <ecto/schedulers/singlethreaded.hpp>
#include <ecto/plasm.hpp>

#define STRINGDIDLY(A) std::string(#A)

using namespace ecto;
namespace {
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

  struct Passthrough
  {
    static void declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      in.declare<tendril::none>("in", "Any type");
      out.declare<tendril::none>("out", "Any type on the output...");
      out["out"] = in["in"]; //assign the ptr.
    }
  };

}
TEST(Plasm, Viz)
{
  ecto::plasm p;
  ecto::cell::ptr m1(new ecto::cell_<Module1>), m2(new ecto::cell_<Module2>);
  m1->declare_params();
  m1->declare_io();
  m2->declare_params();
  m2->declare_io();
  p.connect(m1,"d",m2,"d");
  std::cout << p.viz() << std::endl;
}

TEST(Plasm, Passthrough)
{
  ecto::plasm::ptr p(new ecto::plasm);
  ecto::cell::ptr m1(new cell_<Module1>), 
    m2(new cell_<Module2>), 
    pass(new cell_<Passthrough>);
  m1->declare_params();
  m2->declare_params();
  pass->declare_params();
  m1->declare_io();
  m2->declare_io();
  pass->declare_io();
  m1->outputs["d"] << 5.0;
  p->connect(m1,"d",pass,"in");
  p->connect(pass,"out",m2,"d");
  ecto::schedulers::singlethreaded sched(p);
  sched.execute(1);
  double out;
  m2->inputs["d"] >> out;
  EXPECT_TRUE(out == 5.0);
  pass->outputs["out"] >> out;
  EXPECT_TRUE(out == 5.0);
}

TEST(Plasm,Registry)
{
  ecto::cell::ptr add = ecto::registry::create("ecto_test::Add");
  EXPECT_TRUE(add);
  std::cout << add->name() << std::endl;
}




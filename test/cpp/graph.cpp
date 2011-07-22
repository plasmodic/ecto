#include <gtest/gtest.h>
#include <ecto/ecto.hpp>
#include <ecto/plasm.hpp>

#define STRINGDIDLY(A) std::string(#A)

using namespace ecto;
struct Module1
{
  static void
  declare_io(const tendrils& p, tendrils& in, tendrils& out)
  {
    SHOW();
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
    out.declare<tendril::none>("out", "Any type");
  }
  void configure(const tendrils& parms, tendrils& in, tendrils& out)
  {
    in_ = in["in"];
    out_ = out["out"];
  }
  int process(tendrils& in, tendrils& out)
  {
    out_ << in_;
    return ecto::OK;
  }
  tendril::ptr in_, out_;
};

TEST(Plasm, Viz)
{
  ecto::plasm p;
  ecto::cell::ptr m1 = ecto::create_cell<Module1>(), m2 = ecto::create_cell<Module2>();
  p.connect(m1,"d",m2,"d");
  std::cout << p.viz() << std::endl;
}

TEST(Plasm, Passthrough)
{
  ecto::plasm::ptr p(new ecto::plasm);
  ecto::cell::ptr m1 = ecto::create_cell<Module1>(), 
    m2 = ecto::create_cell<Module2>(), 
    pass = ecto::create_cell<Passthrough>();
  m1->outputs["d"] << 5.0;
  p->connect(m1,"d",pass,"in");
  p->connect(pass,"out",m2,"d");
  p->execute(1);
  double out;
  m2->inputs["d"] >> out;
  EXPECT_TRUE(out == 5.0);
  pass->outputs["out"] >> out;
  EXPECT_TRUE(out == 5.0);
}

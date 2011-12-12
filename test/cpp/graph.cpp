//
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
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

TEST(Plasm, Registry)
{
  ecto::cell::ptr add = ecto::registry::create("ecto_test::Add");
  EXPECT_TRUE(add);
  std::cout << add->name() << std::endl;
  EXPECT_EQ("ecto_test::Add", add->name());
}




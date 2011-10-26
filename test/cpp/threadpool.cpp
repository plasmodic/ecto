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
#include <ecto/plasm.hpp>

#include <ecto/schedulers/multithreaded.hpp>

using namespace ecto;

namespace {

  plasm::ptr makeplasm()
  {
    plasm::ptr p(new plasm);

    cell::ptr ping(registry::create("ecto_test::Ping"));
    ping->declare_params();
    ping->declare_io();

    cell::ptr sleep0(registry::create("ecto_test::Sleep"));
    cell::ptr sleep1(registry::create("ecto_test::Sleep"));
    sleep0->declare_params();
    sleep0->parameters["seconds"] << 0.1;
    sleep0->declare_io();

    sleep1->declare_params();
    sleep1->parameters["seconds"] << 0.1;
    sleep1->declare_io();

    p->connect(ping, "out", sleep0, "in");
    p->connect(sleep0, "out", sleep1, "in");
    return p;
  }

}
TEST(Threadpool, CreateAndDestroy)
{
  plasm::ptr p = makeplasm();
  schedulers::multithreaded* tp = new schedulers::multithreaded(p);
  delete tp;
}


TEST(Multithreaded, DestroyWhileRunning)
{
  plasm::ptr p = makeplasm();
  schedulers::multithreaded* tp = new schedulers::multithreaded(p);
  tp->execute_async();
  delete tp;
}

TEST(Multithreaded, DestroyAfterRunning)
{
  plasm::ptr p = makeplasm();
  schedulers::multithreaded* tp = new schedulers::multithreaded(p);
  tp->execute(2,2);
  EXPECT_FALSE(tp->running());
  delete tp;
}

TEST(Multithreaded, WaitAndDestroy)
{
  plasm::ptr p = makeplasm();
  schedulers::multithreaded* tp = new schedulers::multithreaded(p);
  tp->execute_async(2,4);
  EXPECT_TRUE(tp->running());
  tp->wait();
  EXPECT_FALSE(tp->running());
  delete tp;
}

TEST(Multithreaded, StopAndDestroy)
{
  plasm::ptr p = makeplasm();
  schedulers::multithreaded* tp = new schedulers::multithreaded(p);
  tp->execute_async(2,4);
  EXPECT_TRUE(tp->running());
  tp->stop();
  delete tp;
}


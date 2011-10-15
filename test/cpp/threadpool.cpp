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


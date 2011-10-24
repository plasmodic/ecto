#include <gtest/gtest.h>
#include <boost/asio.hpp>
#include <ecto/all.hpp>
#include <ecto/plasm.hpp>
#include <ecto/atomic.hpp>
#include <ecto/schedulers/multithreaded.hpp>
#include <boost/exception/diagnostic_information.hpp>

namespace bp = boost::python;

using namespace ecto;

namespace {

  boost::mutex mtx;
  boost::asio::io_service s;

  struct Crashy
  {
    boost::asio::deadline_timer dt;

    Crashy() : dt(s) { }
    int process(const ecto::tendrils&, const ecto::tendrils&)
    {
      boost::mutex::scoped_try_lock lock(mtx);
      ECTO_ASSERT(lock.owns_lock(), "we should own this lock");
      dt.expires_from_now(boost::posix_time::milliseconds(200));
      dt.wait();
      return ecto::OK;
    }
  };

}

ECTO_THREAD_UNSAFE(Crashy);

TEST(Strands, Strandy)
{
  ecto::plasm::ptr p(new ecto::plasm);
  for (unsigned j=0; j<10; ++j) {
    ecto::cell::ptr m(new cell_<Crashy>);
    p->insert(m);
  }

  ecto::schedulers::multithreaded sched(p);
  sched.execute(5);
}

namespace
{
  ecto::atomic<int> n_concurrent(0), max_concurrent(0);

  struct NotCrashy
  {
    boost::asio::deadline_timer dt;

    NotCrashy() : dt(s) { }
    int process(const ecto::tendrils&, const ecto::tendrils&)
    {
      {
        ecto::atomic<int>::scoped_lock nconc(n_concurrent), maxconc(max_concurrent);
        ++nconc.value;
        if (maxconc.value < nconc.value)
          maxconc.value = nconc.value;
      }

      dt.expires_from_now(boost::posix_time::milliseconds(200));
      dt.wait();

      {
        ecto::atomic<int>::scoped_lock nconc(n_concurrent);
        --nconc.value;
      }
      return ecto::OK;
    }
  };

}


TEST(Strands, ConcurrencyCount)
{
  ecto::plasm::ptr p(new ecto::plasm);
  for (unsigned j=0; j<10; ++j) {
    ecto::cell::ptr m(new cell_<NotCrashy>);
    p->insert(m);
  }

  ecto::schedulers::multithreaded sched(p);
  sched.execute(boost::thread::hardware_concurrency()+8);
  ecto::atomic<int>::scoped_lock cur_con(n_concurrent), max_con(max_concurrent);
  ASSERT_EQ(cur_con.value, 0);
  ASSERT_EQ(max_con.value, boost::thread::hardware_concurrency());
  ECTO_LOG_DEBUG("max concurrent runs: %u", max_con.value);
}



TEST(Strands, Registry) {
  ecto::cell_ptr cp = ::ecto::registry::create("ecto_test::CantCallMeFromTwoThreads");
  ASSERT_TRUE(cp->strand_);

  ecto::cell_ptr cpother = ::ecto::registry::create("ecto_test::CantCallMeFromTwoThreads");
  ASSERT_TRUE(cpother->strand_);

  ASSERT_TRUE(*(cp->strand_) == *(cpother->strand_));

  ecto::cell_ptr cp2 = ::ecto::registry::create("ecto_test::DontCallMeFromTwoThreads");
  ASSERT_FALSE(cp2->strand_);


}

#include <stdlib.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/atomic.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/shared_array.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/exception/all.hpp>
#include <iostream>
#include <deque>

//  N worker threads running some function for a while
//  sometimes they throw
//  when they throw, main thread propagates the exception

using namespace boost;

struct thrower
{
  exception_ptr eptr;
  thrower(exception_ptr eptr_) : eptr(eptr_) { }

  void operator()() const
  {
    std::cout << "thrower is rethrowing" << std::endl;
    rethrow_exception(eptr);
  }
};

struct worker 
{
  asio::io_service& serv;
  asio::io_service::work work;

  unsigned ms;

  worker(asio::io_service& serv_, unsigned ms_) 
    : serv(serv_), work(serv), ms(ms_) 
  { }

  void operator()()
  {
    asio::io_service serv;
    asio::deadline_timer dt(serv);
    for (unsigned j=0; j<4; ++j)
      {
        std::cout << this_thread::get_id() << " " << j << "\n";
        dt.expires_from_now(posix_time::milliseconds(ms));
        dt.wait();
      }
    BOOST_THROW_EXCEPTION(std::runtime_error("OMGTHROW"));
  }

  template <typename Handler>
  void post(Handler h)
  {
    serv.post(h);
  }
};

struct propagator
{
  asio::io_service &from, &to;
  asio::io_service::work work;

  propagator(asio::io_service& from_, asio::io_service& to_) 
  : from(from_), to(to_), work(to) { }

  void operator()() {
    from.run();
  }

  template <typename Handler>
  void post(Handler h)
  {
    to.post(h);
  }
};


struct runandjoin : enable_shared_from_this<runandjoin>
{
  typedef shared_ptr<runandjoin> ptr;

  thread runner;

  runandjoin() { }

  ~runandjoin() {
    std::cout << this << " " << __PRETTY_FUNCTION__ << "\n";
  }

  void joinit(ptr this_) 
  {
    std::cout << this << " " << __PRETTY_FUNCTION__ << "\n";
    runner.join();
  }

  template <typename Work>
  void impl(Work w, ptr this_)
  {
    std::cout << __PRETTY_FUNCTION__ << "\n";
    try {
      w();
    } catch (const exception& e) {
      std::cout << "runandjoin caught:" << diagnostic_information(e) << std::endl;
      w.post(thrower(current_exception()));
    }
    w.post(bind(&runandjoin::joinit, this, shared_from_this()));
  }

  template <typename Work>
  void run(Work w)
  {
    std::cout << this << " " << __PRETTY_FUNCTION__ << "\n";
    thread* newthread = new thread(bind(&runandjoin::impl<Work>, this, w, shared_from_this()));
    newthread->swap(runner);
  }
};

int main()
{
  std::cout << "start..." << std::endl;

  boost::asio::io_service workserv, topserv;

  for (unsigned j=0; j<5; ++j)
    {
      runandjoin::ptr rj(new runandjoin);
      rj->run(worker(workserv, (j+1) * 150 + (j*15)));
    }

  std::cout << "running props...\n";
  for (unsigned j=0; j<3; ++j)
    {
      //      prop::ptr pp(new prop(workserv, topserv));
      //      pp->run();
      runandjoin::ptr rj(new runandjoin);
      rj->run(propagator(workserv, topserv));
    }
  std::cout << "running tops...\n";

  try {
    topserv.run();
  } catch (const exception& e) {
    std::cout << "SMELLS LIKE VICTORY: " << diagnostic_information(e) << "\n";
  }
  std::cout << "exit." << std::endl;
}

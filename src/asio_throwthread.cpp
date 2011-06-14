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

void worker() 
{
  static unsigned offset = 0;
  offset += 5;
  asio::io_service serv;
  asio::deadline_timer dt(serv);
  for (unsigned j=0; j<4; ++j)
    {
      std::cout << this_thread::get_id() << " " << j << "\n";
      dt.expires_from_now(posix_time::milliseconds(500 + offset));
      dt.wait();
    }
  BOOST_THROW_EXCEPTION(std::runtime_error("OMGTHROW"));
}

struct thrower {
  exception_ptr eptr;
  thrower(exception_ptr eptr_) : eptr(eptr_) { }

  void operator()() {
    std::cout << "thrower is rethrowing\n";
    rethrow_exception(eptr);
  }
};

struct runandjoin : enable_shared_from_this<runandjoin>
{
  typedef shared_ptr<runandjoin> ptr;
  thread runner;
  asio::io_service& serv;
  asio::io_service::work work;
  runandjoin(asio::io_service& serv_) : serv(serv_), work(serv)
  { 
    std::cout << this << " " << __PRETTY_FUNCTION__ << "\n";
  }

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
      thrower th(current_exception());
      serv.post(th);
      return;
    }
    serv.post(bind(&runandjoin::joinit, this, shared_from_this()));
  }

  template <typename Work>
  void run(Work w)
  {
    std::cout << this << " " << __PRETTY_FUNCTION__ << "\n";
    thread* newthread = new thread(bind(&runandjoin::impl<Work>, this, w, shared_from_this()));
    newthread->swap(runner);
  }
};

struct prop : enable_shared_from_this<prop> 
{
  typedef shared_ptr<prop> ptr;

  asio::io_service &from, &to;
  asio::io_service::work work;
  thread runner;

  prop(asio::io_service& from_, asio::io_service& to_) : from(from_), to(to_), work(to) { }
  
  void impl(ptr this_)
  {
    try {
      std::cout << this << " running from\n";
      from.run();
      std::cout << this << " done running from\n";
    } catch (const exception& e) {
      std::cout << "POST RETHROWER\n";
      // post rethrower to 'to' serv
    }
    to.post(bind(&prop::joinit, this, shared_from_this()));

  }

  void joinit(ptr this_) 
  {
    std::cout << this << " " << __PRETTY_FUNCTION__ << "\n";
    runner.join();
  }

  void run() 
  {
    thread* newthread = new thread(bind(&prop::impl, this, shared_from_this()));
    newthread->swap(runner);
  }

  ~prop() {
    std::cout << this << " " << __PRETTY_FUNCTION__ << "\n";
  }
};

int main()
{
  std::cout << "start..." << std::endl;

  boost::asio::io_service workserv, topserv;

  for (unsigned j=0; j<5; ++j)
    {
      runandjoin::ptr rj(new runandjoin(workserv));
      rj->run(&worker);
    }

  std::cout << "running props...\n";
  for (unsigned j=0; j<3; ++j)
    {
      prop::ptr pp(new prop(workserv, topserv));
      pp->run();
    }

  std::cout << "running tops...\n";
  topserv.run();
  std::cout << "exit." << std::endl;
}

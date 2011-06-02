#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/atomic.hpp>
#include <boost/make_shared.hpp>
#include <boost/lockfree/fifo.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/shared_array.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <deque>

// ----- I/O object

template <typename T>
struct tendril : boost::noncopyable 
{
  typedef boost::shared_ptr<T> pointer_type;
  typedef boost::shared_ptr<const T> const_pointer_type;
  std::deque<pointer_type> deque;
  typedef uint64_t counter_type;
  counter_type counter;
  boost::mutex mtx;
  boost::condition_variable cond;
  tendril() : counter(0) { }
  void push_back(const pointer_type& item)
  {
    boost::unique_lock<boost::mutex> lock(mtx);
    ++counter;
    deque.push_back(item);
  }
  const_pointer_type pop()
  {
    boost::unique_lock<boost::mutex> lock(mtx);
    const_pointer_type front = deque.front();
    deque.pop_front();
    return front;
  }
};

tendril<std::string> box;

class tendril_monitor
{
  boost::asio::io_service& serv;
  boost::scoped_ptr<boost::thread> workthread;

public:
  tendril_monitor(boost::asio::io_service &serv_)
    : serv(serv_)
  { }

  template <typename Handler, typename Tendril>
  void async_wait(Handler handler, Tendril& t)
  {
    std::cout << __PRETTY_FUNCTION__ << "\n";
    workthread.reset(new boost::thread(wait_op<Handler, Tendril>(handler, serv, t)));
  }

  void stop()
  {
    if (!workthread)
      return;
    workthread->interrupt();
    workthread->join();
    workthread.reset();
  }

  ~tendril_monitor() { stop(); }

private:

  template <typename Handler, typename Tendril>
  struct wait_op
  {
    Handler handler_;
    boost::asio::io_service& serv;
    boost::asio::io_service::work work;
    Tendril& tendril;

    wait_op(Handler h,
            boost::asio::io_service& serv_, 
            Tendril& t)
      : handler_(h), serv(serv_), work(serv), tendril(t)
    { };

    ~wait_op() { std::cout << "work disappearing\n"; }

    void operator()()
    {
      typename Tendril::counter_type cntr(0);
      for(;;) 
        {
          boost::unique_lock<boost::mutex> lock(tendril.mtx);
          while(cntr >= tendril.counter)
            tendril.cond.wait(lock);
          cntr = tendril.counter;
          std::cout << "Posting handler..." << std::endl;
          serv.post(boost::bind(handler_, boost::ref(tendril)));
          std::cout << "posted." << std::endl;
        }
    }
  };
};

template <typename Tendril>
void 
wait_handler(Tendril& t)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  std::cout << "SIZE=" << t.deque.size() << " BACK=" << t.deque.back() << std::endl;
  typename Tendril::const_pointer_type p = t.pop();
  std::cout << "SIZE=" << t.deque.size() << "\n";
  std::cout << *p << "\n";
}

template <typename Tendril>
void 
incbox(Tendril& tendril, const boost::system::error_code& ec) 
{
  std::cout << __PRETTY_FUNCTION__ << " " << ec << std::endl;
  {
    boost::shared_ptr<std::string> s(new std::string);
    *s = "<<<" + boost::lexical_cast<std::string>(tendril.counter) + ">>>";
    tendril.push_back(s);
  }
  tendril.cond.notify_all();
}

struct module 
{
  typedef boost::shared_ptr<module> ptr;
  std::map<std::string, boost::shared_ptr<tendril<std::string> > > inputs;
  std::map<std::string, boost::shared_ptr<tendril<std::string> > > outputs;
};

// std::pair<module::ptr, boost::

int main()
{
  std::cout << "start..." << std::endl;
  boost::asio::io_service io_service;
  tendril_monitor monitor(io_service);
  monitor.async_wait(&wait_handler<tendril<std::string> >, box);

  boost::asio::deadline_timer t(io_service), t2(io_service), t3(io_service), t4(io_service);
  t.expires_from_now(boost::posix_time::seconds(2));
  boost::function<void(const boost::system::error_code&)> fn;
  fn = boost::bind(&incbox<tendril<std::string> >, boost::ref(box), _1);
  t.async_wait(fn);
  t2.expires_from_now(boost::posix_time::seconds(3));
  t2.async_wait(fn);
  t3.expires_from_now(boost::posix_time::seconds(4));
  t3.async_wait(fn);
  
  t4.expires_from_now(boost::posix_time::seconds(6));
  t4.async_wait(boost::bind(&tendril_monitor::stop, boost::ref(monitor)));
  std::cout << "run..." << std::endl;
  io_service.run();
  std::cout << "exit." << std::endl;
}

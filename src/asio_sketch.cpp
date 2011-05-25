#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <boost/shared_array.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

// ----- I/O object

template <typename T>
struct tendril : boost::noncopyable 
{
  T value;
  typedef uint64_t counter_type;
  counter_type counter;
  boost::mutex mtx;
  boost::condition_variable cond;
  tendril() : value(T()), counter(0) { }
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
    workthread->interrupt();
    workthread->join();
  }

  ~tendril_monitor() { stop(); }

private:

  template <typename Handler, typename Tendril>
  struct wait_op
  {
    Handler handler_;
    boost::asio::io_service& serv;
    Tendril& tendril;

    wait_op(Handler h,
            boost::asio::io_service& serv_, 
            Tendril& t)
      : handler_(h), serv(serv_), tendril(t)
    { };

    ~wait_op() { std::cout << __PRETTY_FUNCTION__ << std::endl; }

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
wait_handler(Tendril const& t)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  std::cout << "VALUE=" << t.value << std::endl;
}

template <typename Tendril>
void incbox(Tendril& tendril, const boost::system::error_code& ec) 
{
  std::cout << __PRETTY_FUNCTION__ << " " << ec << std::endl;
  {
    boost::unique_lock<boost::mutex> lock(tendril.mtx);
    ++tendril.counter;
    tendril.value = std::string("TENDRILVALUE:::") 
      + boost::lexical_cast<std::string>(tendril.counter);
  }
  tendril.cond.notify_all();
}

int main()
{
  std::cout << "start..." << std::endl;
  boost::asio::io_service io_service;
  tendril_monitor monitor(io_service);
  monitor.async_wait(&(wait_handler<tendril<std::string> >), box);

  boost::asio::deadline_timer t(io_service), t2(io_service);
  t.expires_from_now(boost::posix_time::seconds(2));
  boost::function<void(const boost::system::error_code&)> fn;
  fn = boost::bind(&incbox<tendril<std::string> >, boost::ref(box), _1);
  t.async_wait(fn);
  t2.expires_from_now(boost::posix_time::seconds(3));
  t2.async_wait(fn);


  std::cout << "run..." << std::endl;
  io_service.run();
  std::cout << "exit." << std::endl;
}

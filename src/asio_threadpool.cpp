#include <stdlib.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
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

boost::atomic<unsigned> nsimultaneous, noutstanding;


namespace asio = boost::asio;
asio::io_service io_service;
boost::thread_group tgroup;

/*
void do_work(const boost::system::error_code& ec)
{
  std::cout << "WORKING: " << boost::this_thread::get_id() << "\n";
}

struct workstuff : boost::enable_shared_from_this<workstuff>
{
  typedef boost::shared_ptr<workstuff> ptr;

  boost::asio::io_service& serv_;
  boost::asio::deadline_timer timer;
  unsigned jobnumber;
  
  workstuff(boost::asio::io_service& serv) 
    : serv_(serv), timer(serv_)
  { 
  }

  void start(unsigned s)
  {
    timer.expires_from_now(boost::posix_time::seconds(s));
    timer.async_wait(boost::bind(&workstuff::cb, 
                                 shared_from_this(),
                                 _1));
    jobnumber = noutstanding;
    ++noutstanding;
  }

  static void cb(ptr p, const boost::system::error_code& ec)
  {
    ++nsimultaneous;
    boost::thread::id id = boost::this_thread::get_id();
    unsigned ms_work = rand() % 3000;
    std::cout << ">>> " << id << "\nsimultaneous = " << std::dec << nsimultaneous 
              << " work(" << ms_work << "ms)" << std::endl;

    // spend some time working
    boost::asio::io_service inner_serv;
    boost::asio::deadline_timer dt(inner_serv);
    dt.expires_from_now(boost::posix_time::milliseconds(ms_work));
    dt.wait();

    --nsimultaneous;
    --noutstanding;
    std::cout << "<<< " << id << "\noutstanding = " << noutstanding << std::endl;
  }
};
*/
struct data {
  float value;
  typedef boost::shared_ptr<data> ptr;
  typedef boost::shared_ptr<const data> const_ptr;
};

typedef std::deque<data::const_ptr> datadeque;

struct module : boost::enable_shared_from_this<module>
{
  typedef boost::shared_ptr<module> ptr;
  typedef std::map<std::string, datadeque> inputs_t;
  inputs_t inputs;

  typedef std::map<std::string, std::pair<ptr, std::string> > outputs_t;
  outputs_t outputs;

  boost::mutex mtx;

  bool inputs_ready() 
  {
    boost::unique_lock<boost::mutex> lock(mtx);
    if (inputs.size() == 0)
      return true;
    for (inputs_t::iterator it = inputs.begin(), end = inputs.end();
         it != end;
         ++it)
      {
        if (it->second.size() == 0)
          return false;
      }
    return true;
  }

  void push(const std::string& port, data::const_ptr newdata)
  {
    boost::unique_lock<boost::mutex> lock(mtx);
    inputs[port].push_back(newdata);
  }

  void doit(boost::asio::io_service& serv) 
  {
    // the input_watcher is done... that's why we're here.
    // input_watcher->join();

    data::ptr newdata(new data);
    newdata->value = 1;
    {
      boost::unique_lock<boost::mutex> lock(mtx);
      for (inputs_t::iterator it = inputs.begin(), end = inputs.end();
           it != end;
           ++it)
        {
          newdata->value += it->second.front()->value;
          it->second.pop_front();
        }
    }
    std::cout << this << " new value = " << newdata->value << "\n";

    for (outputs_t::iterator it = outputs.begin(), end = outputs.end();
         it != end;
         ++it)
      {
        module::ptr downstream_module = it->second.first;
        const std::string& downstream_port = it->second.second;

        downstream_module->push(downstream_port, newdata);
      }

    run(serv);
  }

  static void async_waitforinput(boost::asio::io_service& serv,
                                 module::ptr m)
  {
    boost::asio::io_service inner_serv;
    boost::asio::deadline_timer dt(inner_serv);

    assert(m.get());
      
    for (;;) {
      if (m->inputs_ready())
        {
          serv.post(boost::bind(&module::doit, m, serv));
          return;
        }
      dt.expires_from_now(boost::posix_time::milliseconds(100));
      dt.wait();
    }
  }
                                 
  boost::scoped_ptr<boost::thread> input_watcher;

  void run(boost::asio::io_service& serv)
  {
    input_watcher.reset(new boost::thread(boost::bind(&async_waitforinput,
                                                      serv, shared_from_this())));
  }
};

module::ptr 
make_graph(boost::asio::io_service& serv, unsigned N)
{
  module::ptr root(new module);
  root->run(serv);
  module::ptr prev = root;
  module::ptr next;
  for (unsigned n=0; n<N; ++n)
    {
      next.reset(new module);
      next->run(serv);
      prev->outputs["out"] = make_pair(next, std::string("in"));
      prev = next;
    }
  return root;
}


int main()
{
  using boost::bind;
  namespace asio = boost::asio;

  std::cout << "start..." << std::endl;

  module::ptr graph = make_graph(io_service, 5);

  graph->run(io_service);

  {
    for (unsigned j=0; j<4; ++j)
      {
        tgroup.create_thread(bind(&asio::io_service::run, &io_service));
      }

    /*
    for (unsigned k=1; k<=30; ++k)
      {
        workstuff::ptr w(new workstuff(io_service));
        w->start(k%4);
      }
    */
  }

  tgroup.join_all();
  std::cout << "exit." << std::endl;
}

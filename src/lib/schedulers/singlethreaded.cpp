#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/cell.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

#include <ecto/graph_types.hpp>
#include <ecto/plasm.hpp>
#include <ecto/schedulers/invoke.hpp>
#include <ecto/schedulers/singlethreaded.hpp>

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>


namespace ecto {

  using namespace ecto::graph;
  using boost::scoped_ptr;
  using boost::thread;
  using boost::bind;

  namespace schedulers {

    singlethreaded::singlethreaded(plasm::ptr p) 
      : plasm_(p), graph(p->graph()) 
    {       
      assert(plasm_);
    }

    singlethreaded::singlethreaded(plasm& p) 
      : plasm_(p.shared_from_this()), graph(plasm_->graph()) 
    {       
      assert(plasm_);
    }

    singlethreaded::~singlethreaded()
    {
      interrupt();
      wait();
    }
    int 
    singlethreaded::invoke_process(graph_t::vertex_descriptor vd)
    {
      return ecto::schedulers::invoke_process(graph, vd);
    }

    void singlethreaded::compute_stack()
    {
      if (!stack.empty()) //will be empty if this needs to be computed.
        return;
      //check this plasm for correctness.
      plasm_->check();
      plasm_->configure_all();
      boost::topological_sort(graph, std::back_inserter(stack));
      std::reverse(stack.begin(), stack.end());
    }

    static bool interrupted;
    static void sigint_static_thunk(int)
    {
      std::cerr << "*** SIGINT received, stopping graph execution.\n"
                << "*** If you are stuck here, you may need to hit ^C again\n"
                << "*** when back in the interpreter thread.\n"
                << "*** or Ctrl-\\ (backslash) for a hard stop.\n"
                << std::endl;
      interrupted = true;
    }

    void singlethreaded::execute_async(unsigned niter) {
      boost::mutex::scoped_lock l(iface_mtx);
      running_ = true;
      //compute_stack(); //FIXME hack for python based tendrils.
      scoped_ptr<thread> tmp(new thread(bind(&singlethreaded::execute_impl, this, niter)));
      tmp->swap(runthread);
    }

    int singlethreaded::execute(unsigned niter)
    {
      boost::mutex::scoped_lock l(iface_mtx);
      return execute_impl(niter);
    }

    int singlethreaded::execute_impl(unsigned niter)
    {
      running_ = true;
      interrupted = false;
#if !defined(_WIN32)
      signal(SIGINT, &sigint_static_thunk);
#endif
      compute_stack();
      unsigned cur_iter = 0;
      while((niter == 0 || cur_iter < niter) && running_)
        {
          for (size_t k = 0; k < stack.size(); ++k)
            {
              //need to check the return val of a process here, non zero means exit...
              size_t retval = invoke_process(stack[k]);
              if (retval)
                return retval;
              if(interrupted) return true;
            }
          ++cur_iter;
        }
      running_ = false;
      return 0;
    }

    void singlethreaded::interrupt() {
      boost::mutex::scoped_lock l(iface_mtx);
      runthread.interrupt();
      runthread.join();
      running_ = false;
    }
    void singlethreaded::stop() {
      boost::mutex::scoped_lock l(iface_mtx);
      running_ = false;
    }

    bool singlethreaded::running() const {
      boost::mutex::scoped_lock l(iface_mtx);
      return running_;
    }

    void singlethreaded::wait() {
      boost::mutex::scoped_lock l(iface_mtx);
      while(running_)
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
      runthread.join();
    }

  }
}

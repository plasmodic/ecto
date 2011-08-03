// #define ECTO_TRACE_EXCEPTIONS
#include <Python.h>
#define DISABLE_SHOW
#include <ecto/util.hpp>
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
      int rv;
      try {
        rv = ecto::schedulers::invoke_process(graph, vd);
      } catch (const boost::thread_interrupted& e) {
        std::cout << "Interrupted\n";
        return ecto::QUIT;
      }
      return rv;
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

    void singlethreaded::execute_async(unsigned niter) 
    {
      PyEval_InitThreads();
      assert(PyEval_ThreadsInitialized());

      boost::mutex::scoped_lock l(iface_mtx);
      // compute_stack(); //FIXME hack for python based tendrils.
      scoped_ptr<thread> tmp(new thread(bind(&singlethreaded::execute_impl, this, niter)));
      tmp->swap(runthread);
      while(!running()) usleep(5);
    }

    int singlethreaded::execute(unsigned niter)
    {
      boost::mutex::scoped_lock l(iface_mtx);
      int j;
      j = execute_impl(niter);
      return j;
    }

    int singlethreaded::execute_impl(unsigned niter)
    {
      interrupted = false;
#if !defined(_WIN32)
      signal(SIGINT, &sigint_static_thunk);
#endif
      compute_stack();
      boost::mutex::scoped_lock yes_running(running_mtx);

      unsigned cur_iter = 0;
      while((niter == 0 || cur_iter < niter))
        {
          for (size_t k = 0; k < stack.size() && !interrupted; ++k)
            {
              //need to check the return val of a process here, non zero means exit...
              size_t retval = invoke_process(stack[k]);
              if (retval)
                return retval;
            }
          ++cur_iter;
        }
      return interrupted;
    }

    void singlethreaded::interrupt() {
      SHOW();
      boost::mutex::scoped_lock l(iface_mtx);
      interrupted = true;
      runthread.interrupt();
      usleep(1000);
      runthread.join();
    }
    void singlethreaded::stop() {
      SHOW();
      boost::mutex::scoped_lock l(iface_mtx);
      interrupted = true;
    }

    bool singlethreaded::running() const {
      boost::mutex::scoped_lock l2(running_mtx, boost::defer_lock);
      if (l2.try_lock())
        return false;
      else
        return true;
    }

    void singlethreaded::wait() {
      SHOW();
      boost::mutex::scoped_lock l(iface_mtx);
      while(running())
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
      runthread.join();
    }

  }
}

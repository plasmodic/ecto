#include <boost/scoped_ptr.hpp>
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
#include <ecto/scheduler/invoke.hpp>
#include <ecto/scheduler/singlethreaded.hpp>


namespace ecto {

  using namespace ecto::graph;
  using boost::scoped_ptr;
  using boost::thread;
  using boost::bind;

  namespace scheduler {

    singlethreaded::singlethreaded(plasm& p) 
      : plasm_(p), graph(p.graph()) 
    { }

    int 
    singlethreaded::invoke_process(graph_t::vertex_descriptor vd)
    {
      return ecto::scheduler::invoke_process(graph, vd);
    }

    void singlethreaded::compute_stack()
    {
      if (!stack.empty()) //will be empty if this needs to be computed.
        return;
      //check this plasm for correctness.
      plasm_.check();
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
      running_ = true;
      scoped_ptr<thread> tmp(new thread(bind(&singlethreaded::execute, this, niter)));
      tmp->swap(runthread);
    }

    int singlethreaded::execute(unsigned niter)
    {
      running_ = true;
      interrupted = false;
      signal(SIGINT, &sigint_static_thunk);

      compute_stack();
      unsigned cur_iter = 0;
      while(niter == 0 || cur_iter < niter)
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

    void singlethreaded::stop() {
      running_ = false;
      runthread.join();
    }

    bool singlethreaded::running() const {
      return running_;
    }

    void singlethreaded::wait() {
      while(running_)
        usleep(10);
      runthread.join();
    }

  }
}

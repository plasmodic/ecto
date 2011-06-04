#include <boost/python.hpp>
#include <ecto/scheduler/singlethreaded.hpp>

namespace bp = boost::python;

namespace ecto {
  namespace py {
    void wrapSchedulers()
    {
      bp::class<scheduler::singlethreaded, boost::noncopyable>("singlethreaded", init<) c;
      c
        .def( 
        


    }
  }
}



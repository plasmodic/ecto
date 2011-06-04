#include <boost/python.hpp>
#include <ecto/scheduler/singlethreaded.hpp>

namespace bp = boost::python;

namespace ecto {
  namespace py {
    void wrapSchedulers()
    {
      bp::class_<scheduler::singlethreaded, boost::noncopyable> 
        c("singlethreaded", bp::init<ecto::graph::graph_t&>());
      //      c
      //        .def( 
        


    }
  }
}



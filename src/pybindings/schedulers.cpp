#include <boost/python.hpp>
#include <ecto/scheduler/singlethreaded.hpp>

namespace bp = boost::python;

namespace ecto {
  namespace py {
    void wrapSchedulers()
    {
      bp::class_<scheduler::singlethreaded, boost::noncopyable> 
        c("Singlethreaded", bp::init<ecto::plasm&>());
      c.def("execute", &scheduler::singlethreaded::execute);
    }
  }
}



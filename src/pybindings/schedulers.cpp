#include <boost/python.hpp>
#include <ecto/scheduler/singlethreaded.hpp>
#include <ecto/scheduler/threadpool.hpp>

namespace bp = boost::python;

BOOST_PYTHON_MODULE(schedulers)
{
  using namespace ecto::scheduler;
  using bp::arg;
  {
    bp::class_<singlethreaded, boost::noncopyable> d("Singlethreaded", bp::init<ecto::plasm&>());
    d
      .def("execute", (int (singlethreaded::*)())&singlethreaded::execute)
      .def("execute", (int (singlethreaded::*)(unsigned))&singlethreaded::execute,
           arg("niter"))
      ;
  }
  {
    bp::class_<threadpool, boost::noncopyable> d("Threadpool", bp::init<ecto::plasm&>());
    d
      .def("execute", (int (threadpool::*)())&threadpool::execute)
      .def("execute", (int (threadpool::*)(unsigned))&threadpool::execute,
           arg("nthreads"))
      .def("execute", (int (threadpool::*)(unsigned, unsigned))&threadpool::execute,
           (arg("nthreads"), arg("niter")));
  }
}

namespace ecto {
  namespace py {
    void wrapSchedulers()
    {
      bp::detail::init_module("ecto.schedulers", initschedulers);
      bp::object schedulers_module(bp::handle<>(bp::borrowed(PyImport_AddModule("schedulers"))));

      bp::scope().attr("schedulers") = schedulers_module;
    }
  }
}



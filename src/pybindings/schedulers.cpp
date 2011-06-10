#include <boost/python.hpp>
#include <ecto/scheduler/singlethreaded.hpp>
#include <ecto/scheduler/threadpool.hpp>

namespace bp = boost::python;

BOOST_PYTHON_MODULE(schedulers)
{
  using namespace ecto::scheduler;

  {
    bp::class_<singlethreaded, boost::noncopyable> d("Singlethreaded", bp::init<ecto::plasm&>());
    d
      .def("execute", &singlethreaded::execute);
  }
  {
    bp::class_<threadpool, boost::noncopyable> d("Threadpool", bp::init<ecto::plasm&>());
    d
      .def("execute", (int (threadpool::*)(unsigned))&threadpool::execute)
      .def("execute", (int (threadpool::*)(unsigned, unsigned))&threadpool::execute);
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



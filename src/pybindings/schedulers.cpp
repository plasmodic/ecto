#include <boost/python.hpp>
#include <ecto/scheduler/singlethreaded.hpp>
#include <ecto/scheduler/threadpool.hpp>

namespace bp = boost::python;

BOOST_PYTHON_MODULE(schedulers)
{
  {
    bp::class_<ecto::scheduler::singlethreaded, boost::noncopyable> d("Singlethreaded", bp::init<ecto::plasm&>());
    d
      .def("execute", &ecto::scheduler::singlethreaded::execute);
  }
  {
    bp::class_<ecto::scheduler::threadpool, boost::noncopyable> d("Threadpool", bp::init<ecto::plasm&>());
    d
      .def("execute", &ecto::scheduler::threadpool::execute);
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



#include <boost/python.hpp>
#include <ecto/scheduler/singlethreaded.hpp>
#include <ecto/scheduler/threadpool.hpp>

namespace bp = boost::python;

namespace ecto {
  namespace py {

    template <typename T> int execute0 (T& s) { return s.execute(); }
    template <typename T> int execute1 (T& s, unsigned arg1) { return s.execute(arg1); }
    template <typename T> int execute2 (T& s, unsigned arg1, unsigned arg2) { return s.execute(arg1, arg2); }

    template <typename T> void execute_async0 (T& s) { return s.execute_async(); }
    template <typename T> void execute_async1 (T& s, unsigned arg1) { return s.execute_async(arg1); }
    template <typename T> void execute_async2 (T& s, unsigned arg1, unsigned arg2) 
    { 
      return s.execute_async(arg1, arg2); 
    }

    void wrapSchedulers()
    {
      //      bp::detail::init_module("ecto.schedulers", initschedulers);
      bp::object schedulers_module(bp::handle<>(bp::borrowed(PyImport_AddModule("ecto.schedulers"))));
      bp::scope().attr("schedulers") = schedulers_module;
      bp::scope schedulers_scope = schedulers_module;
      
      using namespace ecto::scheduler;
      using bp::arg;
      bp::class_<singlethreaded, boost::noncopyable>("Singlethreaded", bp::init<ecto::plasm::ptr>())
        .def(bp::init<ecto::plasm&>())
        .def("execute", &execute0<singlethreaded>)
        .def("execute", &execute1<singlethreaded>, arg("niter"))

        .def("execute_async", &execute_async0<singlethreaded>)
        .def("execute_async", &execute_async1<singlethreaded>, arg("niter"))

        .def("stop", &singlethreaded::stop)
        .def("running", &singlethreaded::running)
        .def("wait", &singlethreaded::wait)
        ;

      bp::class_<threadpool, boost::noncopyable>("Threadpool", bp::init<ecto::plasm::ptr>())

        .def(bp::init<ecto::plasm&>())

        .def("execute", &execute0<threadpool>)
        .def("execute", &execute1<threadpool>, arg("niter"))
        .def("execute", &execute2<threadpool>, (arg("niter"), arg("nthreads")))

        .def("execute_async", &execute_async0<threadpool>)

        .def("execute_async", &execute_async1<threadpool>, arg("niter"))
        .def("execute_async", &execute_async2<threadpool>, (arg("niter"), arg("nthreads")))

        .def("stop", &threadpool::stop)
        .def("running", &threadpool::running)
        .def("wait", &threadpool::wait)
        ;
    }
  }
}

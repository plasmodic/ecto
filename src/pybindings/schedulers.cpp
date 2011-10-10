// 
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// 
#include <ecto/schedulers/singlethreaded.hpp>
#include <ecto/schedulers/threadpool.hpp>

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
      
      using namespace ecto::schedulers;
      using bp::arg;
      bp::class_<singlethreaded, boost::noncopyable>("Singlethreaded", bp::init<ecto::plasm::ptr>())
        .def(bp::init<ecto::plasm&>())
        .def("execute", &execute0<singlethreaded>)
        .def("execute", &execute1<singlethreaded>, arg("niter"))

        .def("execute_async", &execute_async0<singlethreaded>)
        .def("execute_async", &execute_async1<singlethreaded>, arg("niter"))

        .def("interrupt", &singlethreaded::interrupt)
        .def("stop", &singlethreaded::stop)
        .def("running", &singlethreaded::running)
        .def("wait", &singlethreaded::wait)
        .def("stats", &singlethreaded::stats)
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
        .def("interrupt", &threadpool::interrupt)
        .def("running", &threadpool::running)
        .def("wait", &threadpool::wait)
        .def("stats", &threadpool::stats)
        ;
    }
  }
}


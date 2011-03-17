#pragma once
//do not include this in ecto lib files, only in client modules

//ecto includes
#include <ecto/module.hpp>
#include <ecto/tendril.hpp>
#include <ecto/plasm.hpp>
#include <ecto/util.hpp>

//boost stuff
#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
namespace ecto {

  template <typename T>
  void wrap(const char* name)
  {
    boost::python::class_<T, boost::python::bases<module>, 
      boost::shared_ptr<T>, boost::noncopyable> thing(name);

    thing
      .def("Process", &T::Process)
      .def("Config", &T::Config)
      .def("Params", &T::Params)
      .staticmethod("Params")
      ;
  }
}

#define ECTO_MODULE(name) BOOST_PYTHON_MODULE(name)

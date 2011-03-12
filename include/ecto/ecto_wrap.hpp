#pragma once
#include <ecto/ecto.hpp>
#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
namespace ecto {

  template <typename T>
  void wrap(const char* name)
  {
    boost::python::class_<T, boost::python::bases<module>, boost::shared_ptr<T>, boost::noncopyable> thing(name);
    thing.def("Config", &T::Config);
  }
}

#define ECTO_MODULE(name) BOOST_PYTHON_MODULE(name)

#pragma once
//do not include this in ecto lib files, only in client modules

//ecto includes
#include <ecto/module.hpp>
#include <ecto/tendril.hpp>
#include <ecto/plasm.hpp>
#include <ecto/tendrils.hpp>
#include <ecto/util.hpp>

//boost stuff
#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>


namespace ecto {
  template<typename T>
  struct doc{
    static std::string doc_str;
    static std::string name;
    static std::string getDoc(){return doc_str;}
    static std::string getName(){return name;}
  };
  template<typename T>
  std::string doc<T>::doc_str;
  template<typename T>
  std::string doc<T>::name;
  template <typename T>
  void wrap(const char* name, std::string doc_str = "A module...")
  {
    boost::python::class_<T, boost::python::bases<module>, 
      boost::shared_ptr<T>, boost::noncopyable> m(name);
    typedef doc<T> docT;
    docT::name = name;
    docT::doc_str = doc_str;

    m
      .def("Params", &T::Params)
      .staticmethod("Params")
      .def("Doc", &docT::getDoc)
      .staticmethod("Doc")
      .def("Name", &docT::getName)
      .staticmethod("Name")
      ;
  }
}

#define ECTO_MODULE(name) BOOST_PYTHON_MODULE(name)

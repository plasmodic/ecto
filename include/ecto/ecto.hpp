#pragma once
//boost python first.
#include <boost/python.hpp>
//boost stuff
#include <boost/shared_ptr.hpp>

//do not include this in ecto lib files, only in client modules

//ecto includes
#include <ecto/module.hpp>
#include <ecto/tendril.hpp>
#include <ecto/plasm.hpp>
#include <ecto/tendrils.hpp>
#include <ecto/util.hpp>
#include <ecto/python/raw_constructor.hpp>

/**
 * \namespace ecto
 * \brief ecto is ...
 *
 * ecto is a plugin architecture / pipeline tool based on boost::python.
 */
namespace ecto
{
/**
 * \internal
 */
template<typename T>
struct doc
{
  static std::string doc_str; //!<a doc string for humans to read in python.
  static std::string name; //!< the name for this type.
  //! get the doc
  static std::string getDoc()
  {
    return doc_str;
  }
  //! get the name
  static std::string getName()
  {
    return name;
  }
};
template<typename T>
std::string doc<T>::doc_str;
template<typename T>
std::string doc<T>::name;

template<typename T>
boost::shared_ptr<ecto::module_<T> > raw_construct(boost::python::tuple args,
    boost::python::dict kwargs)
{
  typedef ecto::module_<T> module_t;

  namespace bp = boost::python;

  //SHOW();
  boost::shared_ptr<module_t> m(new module_t());
  m->module::initialize<T>();
  bp::list l = kwargs.items();
  for (unsigned j = 0; j < bp::len(l); ++j)
  {
    bp::object key = l[j][0];
    bp::object value = l[j][1];
    std::string keystring = bp::extract<std::string>(key);
    std::string valstring = bp::extract<std::string>(value.attr("__repr__")());
    m->parameters.at(keystring).set(value);
  }
  m->module::configure();
  return m;
}

template<typename T>
void wrap(const char* name, std::string doc_str = "A module...")
{
  typedef ecto::module_<T> module_t;
  //SHOW();
  boost::python::class_<module_t, boost::python::bases<module>, boost::shared_ptr<module_t>,
      boost::noncopyable> m(name);
  typedef doc<T> docT;
  docT::name = name;
  docT::doc_str = doc_str;

  m.def("__init__", boost::python::raw_constructor(&raw_construct<T> ));
  m.def("Initialize", &T::Initialize);
  m.staticmethod("Initialize");
  m.def("doc", &docT::getDoc) .staticmethod("doc");
  m.def("name", &docT::getName);
  m.staticmethod("name");
}
}


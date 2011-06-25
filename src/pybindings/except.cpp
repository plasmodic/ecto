#include <ecto/ecto.hpp>
namespace bp = boost::python;
using namespace ecto::except;
namespace ecto
{

  namespace py
  {
    PyObject * pyValueNone, *pyTypeMismatch, *pyValueRequired;
    template<typename ExceptionType>
    struct Translate_
    {
      static void translate(const ExceptionType & x)
      {
        PyErr_SetObject(Exc_Type_, bp::object(x).ptr());
      }
      static PyObject* Exc_Type_;
    };

    template<typename ExceptionType>
    PyObject* Translate_<ExceptionType>::Exc_Type_;

    template<typename ExceptionType>
    void register_exception(const std::string& name)
    {
      bp::class_<ExceptionType> exc_cl(name.c_str());
      Translate_<ExceptionType>::Exc_Type_ = exc_cl.ptr();
      bp::register_exception_translator<ExceptionType>(&Translate_<ExceptionType>::translate);
    }

    void wrap_except()
    {
      register_exception<ValueNone>("ValueNone");
      register_exception<TypeMismatch>("TypeMismatch");
      register_exception<ValueRequired>("ValueRequired");
    }
  }
}

#include <ecto/ecto.hpp>
#include <boost/python/errors.hpp>
namespace bp = boost::python;
using namespace ecto::except;
namespace ecto
{

  namespace py
  {
    template<typename ExceptionType>
    struct Translate_
    {
      static void
      translate(const ExceptionType & x)
      {
        PyErr_SetString(PyExc_RuntimeError, x.what());
      }
      static PyObject* Exc_Type_;
    };

    template<typename ExceptionType>
    PyObject* Translate_<ExceptionType>::Exc_Type_;

    template<typename ExceptionType>
    void
    register_exception(const char* name)
    {
      bp::class_<ExceptionType> exc_cl(name);
      Translate_<ExceptionType>::Exc_Type_ = exc_cl.ptr();
      bp::register_exception_translator<ExceptionType>(&Translate_<ExceptionType>::translate);
    }

    void
    wrap_except()
    {
//      register_exception<EctoException>("EctoException");
//      register_exception<ValueNone>("ValueNone");
//      register_exception<TypeMismatch>("TypeMismatch");
//      register_exception<ValueRequired>("ValueRequired");
    }
  }
}

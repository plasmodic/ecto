#include <boost/python.hpp>
#include <ecto/ecto.hpp>
#include <boost/python/errors.hpp>
#undef BOOST_EXCEPTION_DYNAMIC_TYPEID
#define BOOST_EXCEPTION_DYNAMIC_TYPEID(x) name_of(x)

namespace bp = boost::python;
using namespace ecto::except;
namespace ecto
{

  namespace py
  {
    PyObject* ectoexception;

    template<typename ExceptionType>
    struct Translate_
    {
      static void
      translate(const ExceptionType & x)
      {
        std::string di = except::diagnostic_string(x);
        PyErr_SetString(Exc_Type_, di.c_str());
      }
      static PyObject* Exc_Type_;
    };

    template<typename ExceptionType>
    PyObject* Translate_<ExceptionType>::Exc_Type_;

    template<typename ExceptionType>
    void
    register_exception(const char* name, const char* fullname)
    {
      PyObject* newex = PyErr_NewException(const_cast<char*>(fullname),
                                           ectoexception, /*dict*/NULL);
      Py_INCREF(newex);
      PyModule_AddObject(bp::scope().ptr(), const_cast<char*>(name), newex);

      Translate_<ExceptionType>::Exc_Type_ = newex;
      bp::register_exception_translator<ExceptionType>(&Translate_<ExceptionType>
                                                       ::translate);
    }

    void
    wrap_except()
    {
      ectoexception = PyErr_NewException(const_cast<char*>("ecto.EctoException"),
                                         PyExc_RuntimeError, NULL);
      Py_INCREF(ectoexception);
      PyModule_AddObject(bp::scope().ptr(), "EctoException", ectoexception);

      Translate_<EctoException>::Exc_Type_ = ectoexception;
      bp::register_exception_translator<EctoException>
        (&Translate_<EctoException>::translate)
        ;


#define REGISTER_EXCEPTION(r, data, E) \
      register_exception<E>(BOOST_PP_STRINGIZE(E), "ecto." BOOST_PP_STRINGIZE(E));
      BOOST_PP_SEQ_FOR_EACH(REGISTER_EXCEPTION, ~, ECTO_EXCEPTIONS);

    }
  }
}

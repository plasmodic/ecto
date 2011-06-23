#include <ecto/except.hpp>

namespace ecto
{
  namespace except
  {
    /** Takes a character string describing the error.  */
    TypeMismatch::TypeMismatch(const std::string& msg):msg_(msg)
    {

    }

    TypeMismatch::~TypeMismatch() throw(){}

    /** Returns a C-style character string describing the general cause of
     *  the current error (the same string passed to the ctor).  */
    const char*
    TypeMismatch::what() const throw ()
    {
      return msg_.c_str();
    }

    /** Takes a character string describing the error.  */
    ValueNone::ValueNone(const std::string& msg):msg_(msg)
    {

    }

    ValueNone::~ValueNone() throw(){}

    /** Returns a C-style character string describing the general cause of
     *  the current error (the same string passed to the ctor).  */
    const char*
    ValueNone::what() const throw ()
    {
      return msg_.c_str();
    }
  }
}

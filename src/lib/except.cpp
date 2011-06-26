#include <ecto/except.hpp>

namespace ecto
{
  namespace except
  {

    EctoException::EctoException(const std::string& msg)
        :
          msg_(msg)
    {
    }
    EctoException::~EctoException() throw ()
    {
    }

    EctoException&
    EctoException::operator<<(const std::string& msg) throw ()
    {
      msg_ += "\n" + msg;
      return *this;
    }
    EctoException&
    EctoException::operator<<(EctoException& e) throw ()
    {
      return *this << name_of(typeid(e)) + " : " + e.msg_;
    }

    const char*
    EctoException::what() const throw ()
    {
      return msg_.c_str();
    }

    TypeMismatch::TypeMismatch(const std::string& msg)
        :
          EctoException(msg)
    {

    }

    ValueNone::ValueNone(const std::string& msg)
        :
          EctoException(msg)
    {

    }

    /** Takes a character string describing the error.  */
    ValueRequired::ValueRequired(const std::string& msg)
        :
          EctoException(msg)
    {

    }

  }
}

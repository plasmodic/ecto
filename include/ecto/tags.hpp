#pragma once
#include <boost/python.hpp>
#include <ecto/util.hpp>
#include <boost/shared_ptr.hpp>
#include <numeric>
namespace ecto
{
  namespace constraints
  {
    struct constraint_base
    {
      virtual
      ~constraint_base()
      {
      }
      std::string
      key() const
      {
        return name_of(typeid(*this));
      }

      virtual
      boost::python::object
      extract() const = 0;

      virtual
      boost::shared_ptr<constraint_base>
      clone() const = 0;
    };

    typedef boost::shared_ptr<constraint_base> ptr;
    template<typename T>
    struct constraint: constraint_base
    {
      virtual
      ~constraint()
      {
      }

      const T& value() const{
        return val_;
      }

      boost::python::object
      extract() const
      {
        return boost::python::object(val_);
      }

      T val_;
      typedef boost::shared_ptr<constraint<T> > ptr;
    };

    template<typename T, typename U>
    struct constraint_cloner: constraint<U>
    {
      boost::shared_ptr<constraint_base>
      clone() const
      {
        boost::shared_ptr<constraint_base> p(new T(*thiz()));
        return p;
      }
      const T* thiz() const
      {
        return static_cast<const T*>(this);
      }
    };

  }
}



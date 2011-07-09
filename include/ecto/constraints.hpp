#pragma once
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
    struct constriaint_cloner_: constraint<U>
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

    struct Doc_: constriaint_cloner_<Doc_,std::string>
    {

    };

    inline Doc_ Doc(const std::string& val)
    {
      Doc_ c;
      c.val_ = val;
      return c;
    }

    struct Required_: constriaint_cloner_<Required_,bool>
    {
    };

    inline Required_ Required(bool val = false)
    {
      Required_ c;
      c.val_ = val;
      return c;
    }

    struct Dynamic_: constriaint_cloner_<Dynamic_,bool>
    {
    };

    inline Dynamic_ Dynamic(bool val = false)
    {
      Dynamic_ c;
      c.val_ = val;
      return c;
    }

    template<typename T>
    struct Min_: constriaint_cloner_<Min_<T>,T>
    {
    };

    template<typename T>
    Min_<T>
    Min(const T& val = std::numeric_limits<T>::min())
    {
      Min_<T> c;
      c.val_ = val;
      return c;
    }

    template<typename T>
    struct Max_: constriaint_cloner_<Max_<T>,T>
    {
    };

    template<typename T>
    Max_<T>
    Max(const T& val = std::numeric_limits<T>::max())
    {
      Max_<T> c;
      c.val_ = val;
      return c;
    }
  }
}



#pragma once
#include <boost/python.hpp>
#include <ecto/util.hpp>
#include <boost/shared_ptr.hpp>
#include <numeric>
namespace ecto
{
  namespace tags
  {
    struct tags_base
    {
      virtual
      ~tags_base()
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
      boost::shared_ptr<tags_base>
      clone() const = 0;
    };

    typedef boost::shared_ptr<tags_base> ptr;
    template<typename T>
    struct tag_: tags_base
    {
      virtual
      ~tag_()
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
      typedef boost::shared_ptr<tag_<T> > ptr;
    };

    template<typename T, typename U>
    struct tag_CRTP_: tag_<U>
    {
      boost::shared_ptr<tags_base>
      clone() const
      {
        boost::shared_ptr<tags_base> p(new T(*thiz()));
        return p;
      }
      const T* thiz() const
      {
        return static_cast<const T*>(this);
      }
    };

    struct tags : std::map<std::string, ptr>
    {
    };

  }
}



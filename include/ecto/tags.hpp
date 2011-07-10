#pragma once
#include <boost/python.hpp>
#include <ecto/util.hpp>
#include <boost/shared_ptr.hpp>
#include <numeric>
#include <map>
#include <string>
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
      const char*
      key() const
      {
        return typeid(*this).name();
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

    struct tags
    {
      void tag(ptr c);
      void tag(const tags_base& c);
      ptr get_tag(const char * key) const;
      ptr get_tag(const tags_base& c) const;
      template <typename T>
      const T& tagged(const tag_<T>& _c) const
      {
        ptr cp = get_tag(_c);
        if(!cp)
          return _c.value();
        return dynamic_cast<tag_<T>&>(*cp).value();
      }
      tags& operator<<(const tags_base& c);
      tags& operator<<(const tags& c);
      typedef std::map<std::string, ptr> tag_map;
      tag_map tags_;
    };

  }
}



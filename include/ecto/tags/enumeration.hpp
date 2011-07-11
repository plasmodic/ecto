#pragma once
#include <ecto/tags.hpp>
#include <vector>
#include <algorithm>
namespace ecto
{
  namespace tags
  {
    template<typename U>
    struct Enumeration_: tag_CRTP_<Enumeration_<U> , std::vector<U> >
    {
      boost::python::object
      extract() const
      {
        boost::python::list l;
        for(size_t i = 0,e = tag_<std::vector<U> >::val_.size(); i < e;i++)
        {
          boost::python::object o(tag_<std::vector<U> >::val_[i]);
          l.append(o);
        }
        return boost::python::object(l);
      }
    };

    template<typename U>
    inline  Enumeration_<U>
    Enumerate(const std::vector<U>& val = std::vector<U>())
    {
      Enumeration_<U> c;
      c.val_ = val;
      return c;
    }

    template <typename T, size_t size>
    inline  Enumeration_<T> Enumerate(T(&array)[size])
    {
      Enumeration_<T> c;
      std::copy(array,array+size,std::back_inserter(c.val_));
      return c;
    }
  }
}

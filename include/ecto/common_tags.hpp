#pragma once
#include <ecto/tags.hpp>
namespace ecto
{
  namespace constraints
  {

    struct Doc_: constraint_cloner<Doc_, std::string>
    {
    };

    inline Doc_
    Doc(const std::string& val)
    {
      Doc_ c;
      c.val_ = val;
      return c;
    }

    struct Required_: constraint_cloner<Required_, bool>
    {
    };

    inline Required_
    Required(bool val = false)
    {
      Required_ c;
      c.val_ = val;
      return c;
    }

    struct Dynamic_: constraint_cloner<Dynamic_, bool>
    {
    };

    inline Dynamic_
    Dynamic(bool val = false)
    {
      Dynamic_ c;
      c.val_ = val;
      return c;
    }

    template<typename T>
    struct Min_: constraint_cloner<Min_<T>, T>
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
    struct Max_: constraint_cloner<Max_<T>, T>
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

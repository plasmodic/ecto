#pragma once
#include <ecto/tags.hpp>
namespace ecto
{
  namespace constraints
  {

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

  }
}

#pragma once
#include <ecto/tags.hpp>
namespace ecto
{
  namespace constraints
  {

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

#pragma once
#include <ecto/tags.hpp>
namespace ecto
{
  namespace constraints
  {

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
  }
}

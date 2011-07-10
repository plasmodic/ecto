#pragma once
#include <ecto/tags.hpp>
namespace ecto
{
  namespace constraints
  {

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
  }
}

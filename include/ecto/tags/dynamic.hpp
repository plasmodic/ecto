#pragma once
#include <ecto/tags.hpp>
namespace ecto
{
  namespace tags
  {

    struct Dynamic_: tag_CRTP_<Dynamic_, bool>
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

#pragma once
#include <ecto/tags.hpp>
namespace ecto
{
  namespace tags
  {

    struct Required_: tag_CRTP_<Required_, bool>
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

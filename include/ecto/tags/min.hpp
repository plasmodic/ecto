#pragma once
#include <ecto/tags.hpp>
namespace ecto
{
  namespace tags
  {

    template<typename T>
    struct Min_: tag_CRTP_<Min_<T>, T>
    {
    };

    Min_<double>
    Min(double val = std::numeric_limits<double>::min())
    {
      Min_<double> c;
      c.val_ = val;
      return c;
    }

  }
}

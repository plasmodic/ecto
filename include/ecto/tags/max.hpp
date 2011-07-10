#pragma once
#include <ecto/tags.hpp>
namespace ecto
{
  namespace tags
  {

    template<typename T>
    struct Max_: tag_CRTP_<Max_<T>, T>
    {
    };

    Max_<double>
    Max(double val = std::numeric_limits<double>::max())
    {
      Max_<double> c;
      c.val_ = val;
      return c;
    }

  }
}

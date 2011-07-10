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
  }
}

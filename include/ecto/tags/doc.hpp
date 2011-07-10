#pragma once
#include <ecto/tags.hpp>
namespace ecto
{
  namespace tags
  {

    struct Doc_: tag_CRTP_<Doc_, std::string>
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

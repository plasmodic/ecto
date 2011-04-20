#include <ecto/tendrils.hpp>

namespace ecto
{

  const tendril& tendrils::at(const std::string& name) const
  {
    map_t::const_iterator it = find(name);
    if(it == end())
      throw std::logic_error(name +" does not exist!");
    return it->second;
  }

  tendril& tendrils::at(const std::string& name)
  {
    map_t::iterator it = find(name);
    if(it == end())
      throw std::logic_error(name +" does not exist!");
    return it->second;
  }
}

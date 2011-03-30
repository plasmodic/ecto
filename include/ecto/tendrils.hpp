#pragma once

#include <ecto/tendril.hpp>
#include <string>
#include <sstream>
#include <cstring>
#include <map>
#include <stdexcept>

namespace ecto
{
  class tendrils : public std::map<std::string, tendril>
  {
  public:
    template<typename T>
    tendril& set(const std::string& name, const std::string& doc, const T& default_val = T())
    {
      map_t::iterator it = find(name);
      //if there are no exiting tendrils by the given name,
      //just add it.
      if (it == end())
      {
        it = insert(std::make_pair(name,tendril::make<T>(default_val, doc))).first;
      }
      else // we want to just return the existing tendril (so that modules preconnected don't get messed up)...
      {
        //there is already an existing tendril with the given name
        //check if the types are the same
        if (!it->second.is_type<T> ())
        {
          std::stringstream ss;
          ss << "Your types aren't the same, this could lead to very undefined behavior...";
          ss << " old type = " << (*this)[name].impl_->type_info().name() << " new type = " << typeid(T).name()
              << std::endl;
          throw std::logic_error(ss.str());
        }
      }
      return it->second;
    }
    template <typename T>
    const T& get(const std::string& name) const
    {
      map_t::const_iterator it = (*this).find(name);
      if (it == (*this).end())
      {
        return tendril().get<T>();
      }else
        return it->second.get<T>();
    }

    template <typename T>
    T& get(const std::string& name)
    {
      map_t::iterator it = (*this).find(name);
      if (it == (*this).end())
      {
        return tendril().get<T>();
      }else
        return it->second.get<T>();
    }

  private:
    typedef std::map<std::string, tendril> map_t;
    friend class module;
    friend class plasm;
  };
}

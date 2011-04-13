#pragma once
#include <ecto/tendril.hpp>
#include <string>
#include <sstream>
#include <cstring>
#include <map>
#include <stdexcept>

namespace ecto
{
  class tendrils : public std::map<std::string, tendril>, boost::noncopyable
  {
  public:
    template<typename T>
    void declare(const std::string& name, const std::string& doc = "TODO: doc str me.", const T& default_val = T()) const
    {
      map_t::const_iterator it = find(name);
      //if there are no exiting tendrils by the given name,
      //just add it.
      if (it == end())
      {
        it = const_cast<tendrils*>(this)->insert(std::make_pair(name,tendril::make<T>(default_val, doc))).first;
      }
      else // we want to just return the existing tendril (so that modules preconnected don't get messed up)...
      {
        //there is already an existing tendril with the given name
        //check if the types are the same
        if (!it->second.is_type<T> ())
        {
          std::stringstream ss;
          ss << "Your types aren't the same, this could lead to very undefined behavior...";
          ss << " old type = " << it->second.impl_->type_name() << " new type = " <<  name_of<T>()
              << std::endl;
          throw std::logic_error(ss.str());
        }
      }
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

    template <typename T>
    static T& get(tendrils & t, const std::string& name)
    {
      map_t::iterator it = t.find(name);
      if (it == t.end())
      {
        return tendril().get<T>();
      }else
        return it->second.get<T>();
    }

    const tendril& at(const std::string& name) const
    {
      map_t::const_iterator it = find(name);
      if(it == end())
        throw std::logic_error(name +" does not exist!");
      return it->second;
    }

    tendril& at(const std::string& name)
    {
      map_t::iterator it = find(name);
      if(it == end())
        throw std::logic_error(name +" does not exist!");
      return it->second;
    }

  private:
    typedef std::map<std::string, tendril> map_t;
    friend class module;
    friend class plasm;
  };
}

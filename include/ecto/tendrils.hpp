#pragma once
#include <ecto/tendril.hpp>
#include <string>
#include <sstream>
#include <cstring>
#include <map>
#include <stdexcept>

namespace ecto
{
class tendrils: public std::map<std::string, tendril>, boost::noncopyable
{
public:
  template<typename T>
  void declare(const std::string& name,
      const std::string& doc = "TODO: doc str me.", const T& default_val = T()) const
  {
    const_cast<tendrils*> (this)->operator [](name) = tendril(default_val, doc);
  }

  template<typename T>
  const T& get(const std::string& name) const
  {
    return at(name).get<T> ();
  }

  template<typename T>
  T& get(const std::string& name)
  {
    return at(name).get<T> ();
  }
  template<typename T>
  static T& get(tendrils & t, const std::string& name)
  {
    return t.get<T> (name);
  }

  const tendril& at(const std::string& name) const;
  tendril& at(const std::string& name);

private:
  typedef std::map<std::string, tendril> map_t;
  friend class module;
  friend class plasm;
};
}

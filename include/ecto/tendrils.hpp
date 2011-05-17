#pragma once
#include <ecto/tendril.hpp>
#include <string>
#include <sstream>
#include <cstring>
#include <map>
#include <stdexcept>

namespace ecto
{
/**
 * \brief The tendrils are a collection for the ecto::tendril class, addressable by a string key.
 */
class tendrils: public std::map<std::string, tendril>, boost::noncopyable
{
public:
  /**
   * \brief Declare a tendril, given a symbolic name, doc string and default value
   * @param name
   * @param doc
   * @param default_val
   */
  template<typename T>
  ConstHandle<T> declare(const std::string& name,
      const std::string& doc = "TODO: doc str me.", const T& default_val = T()) const
  {
    map_t::const_iterator it = find(name);
    //if there are no exiting tendrils by the given name,
    //just add it.
    if (it == end())
    {
      const_cast<tendrils*> (this)->insert(
          std::make_pair(name, tendril(default_val, doc)));
    }
    else // we want to just return the existing tendril (so that modules preconnected don't get messed up)...
    {
      //there is already an existing tendril with the given name
      //check if the types are the same
      if (!it->second.is_type<T> ())
      {
        std::stringstream ss;
        ss
            << "Your types aren't the same, this could lead to very undefined behavior...";
        ss << " old type = " << it->second.type_name() << " new type = "
            << name_of<T> () << std::endl;
        throw std::logic_error(ss.str());
      }
      //most likely should not overwrite tendril's value this behavior
      //FIXME
      //const_cast<tendril&> (it->second).get<T> () = default_val;
    }
    return at(name);
  }
  template<typename T>
  Handle<T> declare(const std::string& name,
      const std::string& doc = "TODO: doc str me.", const T& default_val = T())
  {
    map_t::const_iterator it = find(name);
    //if there are no exiting tendrils by the given name,
    //just add it.
    if (it == end())
    {
      insert(std::make_pair(name, tendril(default_val, doc)));
    }
    else // we want to just return the existing tendril (so that modules preconnected don't get messed up)...
    {
      //there is already an existing tendril with the given name
      //check if the types are the same
      if (!it->second.is_type<T> ())
      {
        std::stringstream ss;
        ss
            << "Your types aren't the same, this could lead to very undefined behavior...";
        ss << " old type = " << it->second.type_name() << " new type = "
            << name_of<T> () << std::endl;
        throw std::logic_error(ss.str());
      }
      //most likely should not overwrite tendril's value this behavior
      //FIXME
      //const_cast<tendril&> (it->second).get<T> () = default_val;
    }
    return at(name);
  }
  /**
   * \brief get the given type that is stored at the given key.  Will throw if there is a type mismatch.
   * @tparam T The compile time type to attempt to get from the tendrils.
   * @param name The key value
   * @return A const reference to the value, no copy is done.
   */
  template<typename T>
  const T& get(const std::string& name) const
  {
    return at(name).get<T> ();
  }

  /**
   * \brief get the given type that is stored at the given key.  Will throw if there is a type mismatch.
   * @tparam T The compile time type to attempt to get from the tendrils.
   * @param name The key value
   * @return A reference to the value, no copy is done.
   */
  template<typename T>
  T& get(const std::string& name)
  {
    return at(name).get<T> ();
  }

  /**
   * \brief Grabs the tendril at the key.
   * @param name The key for the desired tendril.
   * @return A reference to the tendril.
   */
  const tendril& at(const std::string& name) const;
  /**
   * \brief Grabs the tendril at the key.
   * @param name The key for the desired tendril.
   * @return A reference to the tendril.
   */
  tendril& at(const std::string& name);

private:
  typedef std::map<std::string, tendril> map_t;
};
}

/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include <ecto/tendril.hpp>
#include <ecto/spore.hpp>
#include <boost/thread.hpp>

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
  class ECTO_EXPORT tendrils : public std::map<std::string, tendril::ptr>, boost::noncopyable
  {
  public:

    /**
     * \brief Declare a tendril of a certain type, with only a name, no doc, or default values.
     * @tparam T the type of tendril to declare.
     * @param name The key for the tendril. Must be unique.
     * @return A typed holder for the tendril.
     */
    template<typename T>
    spore<T>
    declare(const std::string& name)
    {
      tendril::ptr t(tendril::make_tendril<T>());
      return declare(name, t);
    }

    /**
     * @see tendrils::declare
     * @param name @see tendrils::declare
     * @param doc The doc string for the tendril.
     * @return @see tendrils::declare
     */
    template<typename T>
    spore<T>
    declare(const std::string& name, const std::string& doc)
    {
      return declare<T>(name).set_doc(doc);
    }

    /**
     * @see tendrils::declare
     * @param name @see tendrils::declare
     * @param doc @see tendrils::declare
     * @param default_val A default value for the tendril.
     * @return @see tendrils::declare
     */
    template<typename T>
    spore<T>
    declare(const std::string& name, const std::string& doc, const T& default_val)
    {
      return declare<T>(name, doc).set_default_val(default_val);
    }

    /**
     * \brief get the given type that is stored at the given key.  Will throw if there is a type mismatch.
     * @tparam T The compile time type to attempt to get from the tendrils.
     * @param name The key value
     * @return A const reference to the value, no copy is done.
     */
    template <typename T>
    const T&
    get(const std::string& name) const
    {
      try
      {
        return at(name)->get<T>();
      }catch(except::TypeMismatch& e)
      {
        e << std::string("  Hint : " ) + "'"+name+"' is of type: " + at(name)->type_name();
        throw e;
      }
    }

#if 0
    /**
     * \brief Performs a non ambiguous read operation on the value held at the given name.
     * This should be the preferred method of reading a value if the intent is to not change it.
     * @param name The key that the tendril is stored at.
     * @return A const reference to the value, no copy is done.
     */
    template <typename T>
    const T&
    read(const std::string& name) const
    {
      try
      {
          return at(name)->get<T>();
      } catch(except::TypeMismatch& e) {
        e << std::string("  Hint : " ) + "'"+name+"' is of type: " + at(name)->type_name();
        throw e;
      }
    }
#endif
    /**
     * \brief get the given type that is stored at the given key.  Will throw if there is a type mismatch.
     * @tparam T The compile time type to attempt to get from the tendrils.
     * @param name The key value
     * @return A reference to the value, no copy is done.
     */
    template <typename T>
      T&
    get(const std::string& name)
    {
      try
      {
        return at(name)->get<T>();
      } catch(except::TypeMismatch& e)
      {
        e << std::string("  Hint : " )+ "'"+name+"' is of type: " + at(name)->type_name();
        throw e;
      }
    }

    /**
     * \brief Grabs the tendril at the key.
     * @param name The key for the desired tendril.
     * @return A reference to the tendril.
     */
    tendril::const_ptr
    at(const std::string& name) const;
    /**
     * \brief Grabs the tendril at the key.
     * @param name The key for the desired tendril.
     * @return A reference to the tendril.
     */
    tendril::ptr
    at(const std::string& name);

    /**
     * \brief Print the tendrils documentation string, in rst format.
     * @param out The stream to print to.
     * @param tendrils_name The name used as a label, for the tendrils.
     */
    void
    print_doc(std::ostream& out, const std::string& tendrils_name) const;

    typedef boost::shared_ptr<tendrils> ptr;
    typedef boost::shared_ptr<const tendrils> const_ptr;

  private:
    /**
     * Runtime declare function.
     * @param name
     * @param t
     * @return
     */
    tendril::ptr
    declare(const std::string& name, tendril::ptr t);

    typedef std::map<std::string, tendril::ptr> map_t;
    mutable boost::mutex mtx;
  };
}

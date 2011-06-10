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
class tendrils: public std::map<std::string, tendril>, boost::noncopyable
{
public:

  template<typename T>
  tendril& declare(const std::string& name, const std::string& doc = "TODO: doc str me.", const T& default_val = T())
  {
    map_t::iterator it = find(name);
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
            ss << "Your types aren't the same, this could lead to very undefined behavior...";
            ss << " old type = " << it->second.type_name() << " new type = " << name_of<T> () << std::endl;
            throw std::logic_error(ss.str());
          }
        else
          {
            it->second = tendril(default_val, doc);
          }
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

  /**
   * \brief Print the tendrils documentation string, in rst format.
   * @param out The stream to print to.
   * @param tendrils_name The name used as a label, for the tendrils.
   */
  void print_doc(std::ostream& out, const std::string& tendrils_name) const;

private:
  typedef std::map<std::string, tendril> map_t;
  mutable boost::mutex mtx;
};
}

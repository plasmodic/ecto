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
#include <boost/python.hpp>
#include <boost/python/type_id.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <ecto/util.hpp> //name_of
#include <ecto/tendril.hpp>
#include <stdexcept>
#include <string>
#include <set>
#include <sstream>
#include <cstring>

namespace ecto
{

  template<typename T>
  struct spore
  {
    spore()
    {
    }
    spore(tendril::ptr t) :
        tendril_(t)
    {
      t->enforce_type<T>();
    }

    inline tendril::ptr p()
    {
      return tendril_.lock();
    }

    inline tendril::const_ptr p() const
    {
      return tendril_.lock();
    }

    spore<T>& set_callback(boost::function<void(T)> cb)
    {
      p()->set_callback(cb);
      return *this;
    }

    spore<T>& set_doc(const std::string& doc)
    {
      p()->set_doc(doc);
      return *this;
    }

    spore<T>& set_default_val(const T& val)
    {
      p()->set_default_val(val);
      return *this;
    }

    bool dirty() const
    {
      return p()->dirty();
    }

    bool user_supplied() const
    {
      return p()->user_supplied();
    }

    T* operator->()
    {
      return &(tendril_.lock()->get<T>());
    }
    const T* operator->() const
    {
      return &(tendril_.lock()->get<T>());
    }
    const T& operator*() const
    {
      return tendril_.lock()->get<T>();
    }
    T& operator*()
    {
      return tendril_.lock()->get<T>();
    }
    boost::weak_ptr<tendril> tendril_;
  };
}

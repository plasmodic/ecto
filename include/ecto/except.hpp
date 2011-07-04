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
#include <stdexcept>
#include <string>
#include <sstream>
#include <ecto/util.hpp>

namespace ecto
{
  namespace except
  {
    struct EctoException: std::exception
    {
      std::string msg_;
    public:
      EctoException(const std::string& msg = "");

      virtual
      ~EctoException() throw ();

      EctoException&
      operator<<(const std::string& msg) throw ();

      EctoException&
      operator<<(EctoException& e) throw ();

      const char*
      what() const throw ();
    };

    struct TypeMismatch: EctoException
    {
    public:
      explicit
      TypeMismatch(const std::string& arg = "");
      ~TypeMismatch() throw (){}
      template<typename T1, typename T2>
      static TypeMismatch
      make(const std::string& msg = "")
      {
        return TypeMismatch(msg + "::" + ecto::name_of<T1>() + " != " + ecto::name_of<T2>());
      }
    };

    struct ValueNone: EctoException
    {
    public:
      explicit
      ValueNone(const std::string& arg = "");
      ~ValueNone() throw (){}

    };

    struct ValueRequired: EctoException
    {
    public:
      explicit
      ValueRequired(const std::string& arg = "");
      ~ValueRequired() throw (){}
    };

    struct NonExistant: EctoException
    {

    public:
      explicit
      NonExistant(const std::string& key, const std::string& arg = "");
      ~NonExistant() throw ();
      std::string key;
    };

  }
}

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
#include <boost/optional.hpp>
#include <boost/exception/all.hpp>

#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/seq/for_each.hpp>

namespace ecto
{
  namespace except
  {
    struct EctoException : virtual std::exception, virtual boost::exception
    {
      virtual const char* what() const throw();
    };

#define ECTO_EXCEPTIONS                                                 \
    (TypeMismatch)(ValueNone)(ValueRequired)(NonExistant)               \
    (FailedFromPythonConversion)(TendrilRedeclaration)(CellException)   \
    (NotConnected)(AlreadyConnected)(NullTendril)

    // here, what() actually returns the type name,  the errinfo tag stuff
    // is used for the more illuminating error information
#define ECTO_DECLARE_EXCEPTION(r, data, T)                              \
    struct T : virtual EctoException                                    \
    {                                                                   \
      const char* what() const throw();                                 \
    };

    BOOST_PP_SEQ_FOR_EACH(ECTO_DECLARE_EXCEPTION, ~, ECTO_EXCEPTIONS);

    std::string diagnostic_string(const EctoException&);

    boost::optional<std::string> diagnostic_string(const EctoException& e,
                                                   const std::string& tag);
  }
}

#define ECTO_EXCEPTION_TAG_NAMES                                        \
  (from_typename)(to_typename)(from_key)(to_key)                        \
  (from_cell)(to_cell)(cpp_typename)(pyobject_repr)(actualtype_hint)    \
  (spore_typename)(diag_msg)(actualkeys_hint)(tendril_key)(cell_name)   \
  (function_name)(hint)(which_tendrils)(prev_typename)(cur_typename)    \
  (type)(what)(when)

    #define ECTO_EXCEPTION_TAG_DECL(r, data, NAME)                      \
    typedef boost::error_info<BOOST_PP_CAT(struct tag_, NAME),          \
                              std::string> NAME;                        \

namespace ecto {
  namespace except {
    BOOST_PP_SEQ_FOR_EACH(ECTO_EXCEPTION_TAG_DECL, ~, ECTO_EXCEPTION_TAG_NAMES)
  }
}


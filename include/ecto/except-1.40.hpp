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
#include <boost/format.hpp>

#include <boost/exception/all.hpp>

#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/seq/for_each.hpp>

namespace ecto
{
  namespace except
  {
    template <class T>
    struct wrap { };

    class error_info_container_impl : public ::boost::exception_detail::error_info_container
    {
      typedef ::boost::exception_detail::type_info_ type_info_;
      typedef ::boost::exception_detail::error_info_base error_info_base; 

    public:

      error_info_container_impl(): count_(0) {}

      ~error_info_container_impl() throw() {}

      void
      set(boost::shared_ptr<error_info_base const> const & x, type_info_ const & typeid_)
      {
        BOOST_ASSERT(x);
        info_[typeid_.name()] = x;
        diagnostic_info_str_.clear();
      }

      boost::shared_ptr<error_info_base const>
      get( type_info_ const & ti ) const
      {
        error_info_map::const_iterator i=info_.find(ti.name());
        if( info_.end()!=i )
          {
            boost::shared_ptr<error_info_base const> const & p = i->second;
#ifndef BOOST_NO_RTTI
            BOOST_ASSERT( BOOST_EXCEPTION_DYNAMIC_TYPEID(*p)==ti );
#endif
            return p;
          }
        return boost::shared_ptr<error_info_base const>();
      }

      char const * diagnostic_information() const
      {
        boost::format fmt("%25s  %s\n");
        if( diagnostic_info_str_.empty() )
          {
            std::ostringstream tmp;
            for( error_info_map::const_iterator i=info_.begin(),end=info_.end(); i!=end; ++i )
              {
                boost::shared_ptr<error_info_base const> const & x = i->second;
                tmp << str(fmt % /*name_of(*/x->tag_typeid_name()/*) */
                           % x->value_as_string());
              }
            tmp.str().swap(diagnostic_info_str_);
          }
        return diagnostic_info_str_.c_str();
      }

    private:

      friend class boost::exception;

      typedef std::map<std::string, boost::shared_ptr<error_info_base const> > error_info_map;
      error_info_map info_;
      mutable std::string diagnostic_info_str_;
      mutable int count_;

      void add_ref() const { ++count_; }
      void release() const { if( !--count_ ) delete this; }
    };

    struct EctoException : virtual std::exception, virtual boost::exception
    {
      virtual const char* what() const throw();
    };

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

#define ECTO_EXCEPTION_TAG_DECL(r, data, NAME)                          \
    struct BOOST_PP_CAT(tag_, NAME);                                    \
    typedef ::boost::error_info<wrap<BOOST_PP_CAT(tag_, NAME)>,         \
                                std::string> NAME;                      \

    BOOST_PP_SEQ_FOR_EACH(ECTO_EXCEPTION_TAG_DECL, ~, ECTO_EXCEPTION_TAG_NAMES);

    struct HACK_HACK_HACK;

  }
}

namespace boost {

#define ECTO_EXCEPTION_TAG_TYPE_NAME_DECL(r, data, NAME)                \
  template <> inline char const*                                        \
  tag_type_name< ::ecto::except::wrap< BOOST_PP_CAT(::ecto::except::tag_, NAME)> >() { \
    return BOOST_PP_STRINGIZE(NAME);                                    \
  }
  BOOST_PP_SEQ_FOR_EACH(ECTO_EXCEPTION_TAG_TYPE_NAME_DECL, ~, ECTO_EXCEPTION_TAG_NAMES);

  namespace exception_detail {

    // this break in to a boost::exception and gets the
    // error_info_container out.  
    template <> struct get_info< ::ecto::except::HACK_HACK_HACK>
    {
      template <typename E>
      static
      refcount_ptr<exception_detail::error_info_container>&
      get(E const& e)
      {
        return e.data_;
      }
    };
  }

  template <class E,class Tag,class T>
  inline
  E const &
  operator<<( E const & x, error_info< ::ecto::except::wrap<Tag>, T> const & v )
  {
    typedef error_info< ::ecto::except::wrap<Tag>, T> error_info_tag_t;
    ::boost::shared_ptr<error_info_tag_t> p( new error_info_tag_t(v) );
    exception_detail::refcount_ptr<exception_detail::error_info_container>& c 
      = exception_detail::get_info< ::ecto::except::HACK_HACK_HACK>::get(x);
    if( !(c.get()) )
      c.adopt(new ::ecto::except::error_info_container_impl);
    c.get()->set(p,BOOST_EXCEPTION_STATIC_TYPEID(error_info_tag_t));
    return x;
  }
}


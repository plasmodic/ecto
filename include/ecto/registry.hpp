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
#include <boost/noncopyable.hpp>

namespace ecto {
  namespace registry {

    template <typename T>
    struct module_registry : boost::noncopyable
    {
      typedef boost::function<void(void)> nullary_fn_t;

      void add(nullary_fn_t f)
      {
        regvec.push_back(f);
      }

      void go()
      {
        for (unsigned j=0; j<regvec.size(); ++j)
          regvec[j]();
      }

      std::vector<nullary_fn_t> regvec;

      static module_registry& instance()
      {
        static module_registry instance_;
        return instance_;
      }

    private:

      module_registry() { }
    };

    template <typename Module, typename T>
    struct registrator {
      const char* name_;
      const char* docstring_;

      explicit registrator(const char* name, const char* docstring) 
        : name_(name), docstring_(docstring) 
      { 
        module_registry<Module>::instance().add(boost::ref(*this));
      }

      void operator()() const 
      {
        ecto::wrap<T>(name_, docstring_);
      }
      const static registrator& inst;

    };
  }
}

//  namespace ecto { namespace tag { struct MODULE; } }                 

#define ECTO_MODULETAG(MODULE) namespace ecto { namespace tag { struct MODULE; } }

#define ECTO_CELL(MODULE, TYPE, NAME, DOCSTRING)                        \
  ECTO_MODULETAG(MODULE)                                                \
  namespace ecto{ namespace registry {                                  \
    template<>                                                          \
    const ::ecto::registry::registrator< ::ecto::tag::MODULE,TYPE>&     \
    ::ecto::registry::registrator< ::ecto::tag::MODULE,TYPE>::inst      \
    (::ecto::registry::registrator< ::ecto::tag::MODULE,TYPE>(NAME, DOCSTRING)); \
  } }
  
#define ECTO_INSTANTIATE_REGISTRY(MODULE)                               \
  ECTO_MODULETAG(MODULE)                                                \
  template class ::ecto::registry::module_registry< ::ecto::tag::MODULE>;

#define ECTO_REGISTER(MODULE)                                           \
    ::ecto::registry::module_registry< ::ecto::tag::MODULE>::instance().go();

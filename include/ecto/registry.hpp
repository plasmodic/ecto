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
#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <ecto/util.hpp>
#include <ecto/python.hpp>

namespace ecto {

  struct cell;

  namespace registry {

    typedef boost::function<boost::shared_ptr<cell>()> factory_fn_t;
    typedef boost::function<void(ecto::tendrils&)> declare_params_t;
    typedef boost::function<void(const ecto::tendrils&, ecto::tendrils&, ecto::tendrils&)> declare_io_t;

    struct entry_t {
      factory_fn_t construct;
      declare_params_t declare_params;
      declare_io_t declare_io;

      boost::shared_ptr<cell> construct_() { return construct(); }
      void declare_params_(ecto::tendrils& t) { declare_params(t); }
      void declare_io_(const ecto::tendrils& p, ecto::tendrils& i, ecto::tendrils& o) 
      { 
        declare_io(p, i, o); 
      }

    };


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
        entry_t e;
        e.construct = &ecto::create_cell<T>;
        typedef ::ecto::cell_<T> cell_t;
        e.declare_params = (void (*)(tendrils&)) &cell_t::declare_params;
        e.declare_io = (void (*)(const tendrils&, tendrils&, tendrils&)) &cell_t::declare_io;
        
        register_factory_fn(name_of<T>(), e);
      }

      void operator()() const 
      {
        ecto::wrap<T>(name_, docstring_);
      }
      const static registrator& inst;

    };
    
    entry_t lookup(const std::string& name);
    boost::shared_ptr<cell> create(const std::string& name);
    void register_factory_fn(const std::string& name, entry_t e);

  }
}

//  namespace ecto { namespace tag { struct MODULE; } }                 

#define ECTO_MODULETAG(MODULE) namespace ecto { namespace tag { struct MODULE; } }
#define ECTO_CELL(MODULE, TYPE, NAME, DOCSTRING)                        \
  ECTO_ASSERT_MODULE_NAME(MODULE)                                       \
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

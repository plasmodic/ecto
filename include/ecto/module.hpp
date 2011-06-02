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

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <ecto/tendril.hpp>
#include <ecto/tendrils.hpp>
#include <ecto/util.hpp>

#include <map>

namespace ecto
{

enum ReturnCode
{
  OK = 0, QUIT = 1,
};

/**
 * \brief ecto::module is c++ interface definition for ecto processing
 * modules.  Subclasses should also implement  Initialize, which
 * will take an ecto::tendrils reference.
 */
struct module: boost::noncopyable
{
  typedef boost::shared_ptr<module> ptr; //!< A convenience pointer typedef

  module();
  ~module();

  void declare_params();
  void declare_io();

  void configure();
  ReturnCode process();
  void destroy();

  /**
   * \brief Mark this module dirty, meaning that it needs to recompute its outputs,
   * as either inputs have changed, or parameters have been changed.
   */
  void mark_dirty();

  /**
   * \brief Mark this module clean, meaning that it should not need to recompute its outputs.
   */
  void mark_clean();

  /**
   * \brief test whether the module is dirty or not.
   * @return true if the module is dirty
   */
  bool dirty() const;

  /**
   * \brief test whether the module is clean or not.
   * @return true if the module is clean, and should not need to be processed
   */
  bool clean() const;

  /**
   * \brief Grab the name of the child class.
   * @return
   */
  virtual std::string name() const = 0;

  tendrils parameters, inputs, outputs;

protected:
  virtual void dispatch_declare_params(tendrils& t) = 0;
  virtual void dispatch_declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) = 0;
  virtual void dispatch_configure(tendrils& params) = 0;
  virtual ReturnCode
  dispatch_process(const tendrils& inputs, tendrils& outputs) = 0;
  virtual void dispatch_destroy() = 0;
  bool dirty_;
};

template<class T>
struct has_f
{
  typedef char yes;
  typedef char (&no)[2];

  template<long I> struct S
  {
  };

  // SFINAE eliminates this when the type of arg is invalid
  template<class U>
  static yes test_declare_params(S<sizeof(&U::declare_params)> );
  // overload resolution prefers anything at all over "..."
  template<class U>
  static no test_declare_params(...);

  template<class U>
  static yes test_declare_io(S<sizeof(&U::declare_io)> );
  template<class U>
  static no test_declare_io(...);

  template<class U>
  static yes test_configure(S<sizeof(&U::configure)> );
  template<class U>
  static no test_configure(...);

  template<class U>
  static yes test_process(S<sizeof(&U::process)> );
  template<class U>
  static no test_process(...);

  template<class U>
  static yes test_destroy(S<sizeof(&U::destroy)> );
  template<class U>
  static no test_destroy(...);

  void existent_fn();
  static void existent_static_fn();
  const static S<sizeof(&has_f<T>::existent_fn)> sarg;
  const static S<sizeof(&has_f<T>::existent_static_fn)> ssarg;

  enum
  {
    declare_params = sizeof(test_declare_params<T> (ssarg)) == sizeof(yes)
  };
  enum
  {
    declare_io = sizeof(test_declare_io<T> (ssarg)) == sizeof(yes)
  };
  enum
  {
    configure = sizeof(test_configure<T> (sarg)) == sizeof(yes)
  };
  enum
  {
    process = sizeof(test_process<T> (sarg)) == sizeof(yes)
  };
  enum
  {
    destroy = sizeof(test_destroy<T> (sarg)) == sizeof(yes)
  };

};

/**
 * \brief module_<T> is a convenience structure for registering an arbitrary class
 * with the the module. This adds a barrier between client code and the module.
 */
template<class Module>
struct module_: module
{
protected:
  template<int I>
  struct int_
  {
  };
  typedef int_<0> not_implemented;
  typedef int_<1> implemented;

  static void declare_params(not_implemented, tendrils& params)
  {
    //SHOW();
  }

  static void declare_params(implemented, tendrils& params)
  {
    Module::declare_params(params);
  }

  void dispatch_declare_params(tendrils& params)
  {
    //this is a none static function. for virtuality.
    declare_params(int_<has_f<Module>::declare_params> (), params);
  }

  static void declare_io(not_implemented, const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    //SHOW();
  }
  static void declare_io(implemented, const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    Module::declare_io(params, inputs, outputs);
  }

  void dispatch_declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    declare_io(int_<has_f<Module>::declare_io> (), params, inputs, outputs);
  }

  void configure(not_implemented, const tendrils& params)
  {
    //SHOW();
  }

  void configure(implemented, tendrils& params)
  {
    thiz->configure(params);
  }

  void dispatch_configure(tendrils& params)
  {
    //the module may not be allocated here, so check pointer.
    if (!thiz)
      {
        thiz.reset(new Module);
      }
    configure(int_<has_f<Module>::configure> (), params);
  }

  ReturnCode process(not_implemented, const tendrils& inputs, tendrils& outputs)
  {
    //SHOW();
    return OK;
  }

  ReturnCode process(implemented, const tendrils& inputs, tendrils& outputs)
  {
    return ReturnCode(thiz->process(inputs, outputs));
  }

  ReturnCode dispatch_process(const tendrils& inputs, tendrils& outputs)
  {
    return process(int_<has_f<Module>::process> (), inputs, outputs);
  }

  void destroy(not_implemented)
  {
    //SHOW();
  }

  void destroy(implemented)
  {
    thiz->destroy();
  }

  void dispatch_destroy()
  {
    destroy(int_<has_f<Module>::destroy> ());
  }

  std::string name() const
  {
    return MODULE_TYPE_NAME;
  }

  boost::shared_ptr<Module> thiz;
  static const std::string MODULE_TYPE_NAME;
};

template<typename Module>
const std::string module_<Module>::MODULE_TYPE_NAME = ecto::name_of<Module>();

/**
 * Create a module from an type that has all of the proper interface functions defined.
 * @return A module ptr.
 */
template<typename T>
module::ptr create_module()
{
  module::ptr p(new module_<T> ());
  p->declare_params();
  p->declare_io();
  return p;
}

}//namespace ecto

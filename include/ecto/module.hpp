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

  void initialize();
  void configure();
  void process();
  void reconfigure();
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
  virtual const std::string& name() const = 0;

  void register_finish_handler(boost::function<void()> handler);

  tendrils parameters, inputs, outputs;

protected:
  virtual void dispatch_initialize(tendrils& t) = 0;
  virtual void dispatch_configure(const tendrils& params, tendrils& inputs,
      tendrils& outputs) = 0;
  virtual void dispatch_process(const tendrils& params, const tendrils& inputs,
      tendrils& outputs) = 0;
  virtual void dispatch_reconfigure(const tendrils& params) = 0;
  virtual void dispatch_destroy() = 0;
  virtual void
  dispatch_register_finish_handler(boost::function<void()> handler) = 0;

  bool dirty_;
};

/**
 * \brief module_<T> is a convenience structure for registering an arbitrary class
 * with the the module. This adds a barrier between client code and the module.
 */
template<class Module>
struct module_: module
{
  void dispatch_initialize(tendrils& t)
  {
    thiz.initialize(t);
  }
  void dispatch_configure(const tendrils& params, tendrils& inputs,
      tendrils& outputs)
  {
    thiz.configure(params, inputs, outputs);
  }
  void dispatch_process(const tendrils& params, const tendrils& inputs,
      tendrils& outputs)
  {
    thiz.process(params, inputs, outputs);
  }
  void dispatch_reconfigure(const tendrils& params)
  {
    thiz.reconfigure(params);
  }
  void dispatch_destroy()
  {
    thiz.destroy();
  }
  const std::string& name() const
  {
    return MODULE_TYPE_NAME;
  }
  void dispatch_register_finish_handler(boost::function<void()> handler)
  {
    thiz.register_finish_handler(handler);
  }
  Module thiz;
  static const std::string MODULE_TYPE_NAME;
};

template<typename Module>
const std::string module_<Module>::MODULE_TYPE_NAME = ecto::name_of<Module>();

/**
 * \brief Default implementations of the module interface.
 */
struct module_interface
{
  /**
   * \brief Use initialize to declare and setup your parameters.
   *
   * @param params The parameters to be declared.
   */
  void initialize(tendrils& params);
  /**
   * \brief configure is called after parameters have been declared and the user has had a chance to initialize them.
   *
   * You should declare your inputs and outputs from within this call.
   *
   * @param params The parameters the module. You may conditionally declare inputs and outputs depending on the parameters.
   * @param in The inputs, to be declared.
   * @param out The outputs, to be declared.
   */
  void configure(const tendrils& params, tendrils& in, tendrils& out);
  /**
   *
   * @param params The parameters to the module. All parameters declared in initialize will exist in this object.
   * @param in The inputs, previously declared in configure. These are const and may not be written to in the exectution of this function.
   * @param out The outputs, previously declared in configure. These are non const and should be written to with a get<T>() expression.
   */
  void process(const tendrils& params, const tendrils& in, tendrils& out);
  /**
   * \brief Called when ever parameters change. This may be used to do light weight work, that does
   * not involve changing the inputs or outputs of the module.
   *
   * This should be implemented by the client
   * @param params The parameters that have changed. Hint - all parameters declared in the initialize function will exist
   * in this tendrils object. Not all of the params have necessarily changed.
   */
  void reconfigure(const tendrils& params);
  /**
   * \brief Called when the module is about to be deallocated.
   *
   * This should be implemented by the client. Will be called before shutdown.
   */
  void destroy();

  /**
   * \brief Call this to request the system to stop.
   */
  void finish();
  /**
   * This is called by the system if they wish to register a callback that signals that the system should finish
   * and exit gracefully.
   * @param handler to be called when finish is called.
   */
  void register_finish_handler(boost::function<void()> handler);
private:
  boost::function<void()> finish_handler_;
};

/**
 * Create a module from an type that has all of the proper interface functions defined.
 * @return A module ptr.
 */
template<typename T>
module::ptr create_module()
{
  module::ptr p(new module_<T> ());
  return p;
}

}//namespace ecto

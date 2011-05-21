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

  tendrils parameters, inputs, outputs;

protected:
  virtual void dispatch_initialize(tendrils& t) = 0;
  virtual void dispatch_configure(const tendrils& params, tendrils& inputs, tendrils& outputs) = 0;
  virtual void dispatch_process(const tendrils& params, const tendrils& inputs, tendrils& outputs) = 0;
  virtual void dispatch_reconfigure(const tendrils& params) = 0;
  virtual void dispatch_destroy() = 0;

  bool dirty_;
};

template<typename Module>
struct module_: module
{
  void dispatch_initialize(tendrils& t)
  {
    thiz.initialize(t);
  }
  void dispatch_configure(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    thiz.configure(params, inputs, outputs);
  }
  void dispatch_process(const tendrils& params, const tendrils& inputs, tendrils& outputs)
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
  Module thiz;
  static const std::string MODULE_TYPE_NAME;
};

template <typename Module>
const std::string module_<Module>::MODULE_TYPE_NAME = ecto::name_of<Module>();


/**
 * \brief Default implementations of the module interface.
 */
struct module_interface
{
  void initialize(tendrils& params);
  void configure(const tendrils& parms, tendrils& in, tendrils& out);
  void process(const tendrils& parms, const tendrils& in, tendrils& out);
  void reconfigure(const tendrils& params);
  void destroy();
  void finish();
  void register_finish_handler(boost::function<void()> handler);
private:
  boost::function<void()> finish_handler_;
};

template<typename T>
module::ptr create_module()
{
  module::ptr p(new module_<T> ());
  return p;
}

}//namespace ecto

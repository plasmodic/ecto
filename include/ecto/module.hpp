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
  typedef boost::function<void(const tendrils& params, tendrils& inputs, tendrils& outputs)> f_conf;
  typedef boost::function<void(const tendrils& params, const tendrils& inputs, tendrils& outputs)> f_proc;

  module(f_conf conf, f_proc proc):configurator(conf),processor(proc),dirty_(true){}
  ~module();

  /**
   *\brief Member function to call the static Initialize, this is a convenience thing.
   */
  template<typename T>
  void initialize()
  {
    T::Initialize(parameters);
  }

  void configure();
  void process();


  /**
   *
   * @param output The key to the output. the output will be the source to the input.
   * @param to The pointer to the module that should be connected.
   * @param input The key to the input tendril in the to module.
   */
  void connect(const std::string& output, ptr to, const std::string& input);

  /**
   * \brief Mark this module dirty or clean
   * @param hmm set dirty to this, true if it should be dirty.
   */
  void dirty(bool hmm);
  /**
   * \brief test whether the module is dirty or not.
   * @return
   */
  bool dirty() const;

  /**
   * \brief Grab the name of the child class.
   * @return
   */
  const std::string& name()
  {
    return name_;
  }
  f_conf configurator;
  f_proc processor;
  tendrils parameters, inputs, outputs;
protected:
  std::string name_;
private:
  bool dirty_;
};

template<typename Module>
struct module_: module
{
  //these allow for non ambiguous init of the functions.
  //typedef void(*Initialize_sig)(const tendrils& params, tendrils& inputs, tendrils& outputs);
  typedef void(Module::*configure_sig)(const tendrils& params, tendrils& inputs, tendrils& outputs);
  typedef void(Module::*process_sig)(const tendrils& params, const tendrils& inputs, tendrils& outputs);
  module_() :
    module(//&Module::Initialize,
        boost::bind(&Module::configure, &m_, _1, _2, _3),
        boost::bind(&Module::process, &m_, _1, _2, _3))
  {
    module::name_ = name_of<Module>();
  }
  Module m_;
};

template<typename T>
module::ptr create_module()
{
  module::ptr p(new module_<T>());
  return p;
}

}//namespace ecto

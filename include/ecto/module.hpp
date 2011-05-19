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
  //tendrils parameters_,inputs_,outputs_;
  //typedef  boost::function<void(tendrils& params)> f_parm;
  typedef boost::function<void(const tendrils& params, tendrils& inputs, tendrils& outputs)> f_conf;
  typedef boost::function<void(const tendrils& params, const tendrils& inputs, tendrils& outputs)> f_proc;

  module(/*f_parm parm,*/f_conf conf, f_proc proc):/*paramenator(parm),*/configurator(conf),processor(proc),dirty_(true){}
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
  std::string name()
  {
    return ecto::name_of(typeid(*this));
  }

  //f_parm paramenator;
  f_conf configurator;
  f_proc processor;
  tendrils parameters, inputs, outputs;
private:
  bool dirty_;
  friend class plasm;
  friend class ModuleGraph;
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

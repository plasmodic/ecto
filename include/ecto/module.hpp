#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

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

  module();
  virtual ~module();

  /**
   *\brief Member function to call the static Initialize, this is a convenience thing.
   *
   *
   */
  template<typename T>
  void initialize()
  {
    T::Initialize(params);
  }

  /**
   * \brief Called after parameters have been set and before process. Inputs and Outputs should be declared in this call.
   */
  virtual void configure();

  /**
   * \brief Called after configure, when ever the module is dirty.
   */
  virtual void process();

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
  virtual std::string name()
  {
    return ecto::name_of(typeid(*this));
  }
private:
  tendrils params_, inputs_, outputs_;
  bool dirty_;
public:
  tendrils& params; //!< internal reference to params
  const tendrils& inputs; //!< internal reference to params, notice how inputs are const.
  tendrils& outputs; //!< internal reference to the outputs, non const.
};

}//namespace ecto

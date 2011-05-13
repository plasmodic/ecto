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
  typedef boost::shared_ptr<module> ptr;
  typedef tendrils tendrils_t;

  module();
  virtual ~module();

  /**
   *
   */
  virtual void process();
  /**
   *
   */
  virtual void configure();

  /**
   *
   */
  template<typename T>
  void initialize()
  {
    T::Initialize(params);
  }

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

  virtual std::string name()
  {
    return ecto::name_of(typeid(*this));
  }
  inline const tendrils& i() const
  {
    return inputs;
  }
  inline const tendrils& o() const
  {
    return outputs;
  }
  inline const tendrils& p() const
  {
    return params;
  }
  inline tendrils& o()
  {
    return outputs;
  }
  inline tendrils& p()
  {
    return params;
  }
private:
  tendrils params_, inputs_, outputs_;
  bool dirty_;
public:
  tendrils& params;
  const tendrils& inputs;
  tendrils& outputs;
};

}//namespace ecto

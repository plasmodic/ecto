#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <ecto/tendril.hpp>
#include <ecto/tendrils.hpp>

#include <map>

namespace ecto
{
namespace py
{
void wrapModule();
}
struct module: boost::noncopyable
{
  typedef boost::shared_ptr<module> ptr;
  typedef tendrils tendrils_t;

  module();
  virtual ~module();

  virtual void Process();
  virtual void Config();

  template<typename T>
  void Initialize()
  {
    T::Params(params);
  }

  void connect(const std::string& output, ptr to, const std::string& input);

  void dirty(bool hmm);
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

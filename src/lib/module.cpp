#include <ecto/module.hpp>
#include <ecto/util.hpp>

namespace ecto
{
  module::module() { }

  module::~module(){ }

  void module::declare_params()
  {
    dispatch_declare_params(parameters);
  }

  void module::declare_io()
  {
    dispatch_declare_io(parameters, inputs, outputs);
  }

  void module::configure()
  {
    dispatch_configure(parameters);
  }

  ReturnCode module::process()
  {
    ReturnCode c = dispatch_process(inputs, outputs);
    return c;
  }

  void module::destroy()
  {
    dispatch_destroy();
  }

}

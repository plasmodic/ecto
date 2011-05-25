#include <ecto/module.hpp>
#include <ecto/util.hpp>

namespace ecto
{
module::module() :
  dirty_(true)
{
}

module::~module()
{
}

void module::declare_params()
{
  dispatch_declare_params(parameters);
}
void module::declare_io()
{
  dispatch_declare_io(parameters, inputs, outputs);
  mark_dirty();
}
void module::configure()
{
  dispatch_configure(parameters);
  mark_dirty();
}
ReturnCode module::process()
{
  ReturnCode c = dispatch_process(inputs, outputs);
  mark_clean();
  return c;
}
void module::destroy()
{
  dispatch_destroy();
}

void module::mark_clean()
{
  dirty_ = false;
}
void module::mark_dirty()
{
  dirty_ = true;
}
bool module::dirty() const
{
  return dirty_;
}
bool module::clean() const
{
  return !dirty_;
}

}

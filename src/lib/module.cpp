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

void module::initialize()
{
  dispatch_initialize(parameters);
}

void module::process()
{
  dispatch_process(parameters, inputs, outputs);
  mark_clean();
}
void module::configure()
{
  dispatch_configure(parameters, inputs, outputs);
  mark_dirty();
}

void module::reconfigure()
{
  dispatch_reconfigure(parameters);
  mark_dirty();
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

void module_interface::initialize(tendrils& params)
{

}
void module_interface::configure(const tendrils& parms, tendrils& in, tendrils& out)
{

}
void module_interface::process(const tendrils& parms, const tendrils& in, tendrils& out)
{

}
void module_interface::reconfigure(const tendrils& params)
{

}
void module_interface::destroy()
{

}
void module_interface::finish()
{
  finish_handler_();
}
void module_interface::register_finish_handler(boost::function<void()> handler)
{
  finish_handler_ = handler;
}

}

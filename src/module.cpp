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
  void module::Process() 
  { 
    //throw std::logic_error("Process not implemented");
  } // noop default
  
  void module::Config() 
  { 
    //throw std::logic_error("Config not implemented");
  } // noop default

  void module::connect(const std::string& out_name, ptr to, const std::string& in_name)
  {

    to->inputs[in_name].connect(outputs[out_name]);
  }

  void module::dirty(bool mark)
  {
    dirty_ = mark;
  }
  bool module::dirty() const
  {
    return dirty_;
  }
}


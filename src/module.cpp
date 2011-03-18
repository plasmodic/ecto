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
    for (connections_t::iterator it = outputs.begin(), end = outputs.end(); it != end; ++it)
    {
      it->second.dirty(mark);
    }
    dirty_ = mark;
  }
  bool module::dirty() const
  {
    for (connections_t::const_iterator it = inputs.begin(), end = inputs.end(); it != end; ++it)
    {
      if (it->second.dirty())
        return true;
    }

    return dirty_;
  }
}


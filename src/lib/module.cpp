#include <ecto/module.hpp>
#include <ecto/util.hpp>

namespace ecto
{
  module::module() :
    dirty_(true),params(params_),inputs(inputs_),outputs(outputs_) {
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
    typedef std::map<std::string,tendril> map_t;
    map_t::const_iterator it = to->inputs.find(in_name);
    map_t::const_iterator out_it = outputs.find(out_name);
    const_cast<tendril&>(it->second).connect(const_cast<tendril&>(out_it->second));
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


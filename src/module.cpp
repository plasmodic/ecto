#include <ecto/module.hpp>
#include <ecto/util.hpp>

namespace ecto
{
  module::module() :
   params(),inputs(),outputs(), dirty_(true)
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
    typedef std::map<std::string,tendril> map_t;
    map_t::const_iterator it = to->inputs.find(in_name);
    map_t::const_iterator out_it = outputs.find(out_name);
    if(!it->second.connected())
    {
      const_cast<tendril&>(it->second).connect(const_cast<tendril&>(out_it->second));
    }else
      throw std::logic_error("These tendrils are already connected.  This is considered an error.  The state of the module has not been affected.");
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


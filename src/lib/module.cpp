#include <ecto/module.hpp>
#include <ecto/util.hpp>

namespace ecto
{
  module::module() :
    dirty_(true){
  }

  module::~module()
  {
  }
  void module::process(const tendrils& parameters,const tendrils& inputs, tendrils& outputs)
  { 
    //throw std::logic_error("process not implemented");
  } // noop default
  
  void module::process(){
    process(parameters_,inputs_,outputs_);

  }
  void module::configure()
  {
    configure(parameters_,inputs_,outputs_);
  }
  void module::configure(const tendrils& parameters, tendrils& inputs, tendrils& outputs)
  { 
    //throw std::logic_error("configure not implemented");
  } // noop default

  void module::connect(const std::string& out_name, ptr to, const std::string& in_name)
  {
    typedef std::map<std::string,tendril> map_t;
    map_t::const_iterator it = to->inputs_.find(in_name);
    map_t::const_iterator out_it = outputs_.find(out_name);
    //allow inputs to be connected to many outputs.
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


#include <ecto/module.hpp>
#include <ecto/util.hpp>

namespace ecto
{
//  module::module() :
//    dirty_(true){
//  }

  module::~module()
  {
  }
  
  void module::process(){
    processor(parameters,inputs,outputs);

  }
  void module::configure()
  {
    configurator(parameters,inputs,outputs);
  }

  void module::connect(const std::string& out_name, ptr to, const std::string& in_name)
  {
    typedef std::map<std::string,tendril> map_t;
    map_t::iterator it = to->inputs.find(in_name);
    map_t::iterator out_it = outputs.find(out_name);
    //allow inputs to be connected to many outputs.
    it->second.connect(out_it->second);
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


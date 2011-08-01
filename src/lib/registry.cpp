#include <ecto/ecto.hpp>
#include <ecto/cell.hpp>
#include <ecto/registry.hpp>


namespace ecto {

  namespace registry {



    namespace {
      std::map<std::string, ffn_t> cellmap;
    }

    void register_factory_fn(const std::string& name, ffn_t fn)
    {
      cellmap[name] = fn;
    }
    
    cell::ptr create(const std::string& name)
    {
      std::map<std::string, ffn_t>::iterator iter = cellmap.find(name);
      if (iter != cellmap.end())
        return iter->second();
      else
        throw std::runtime_error("Could not find cell " + name); 
    }


  }
}

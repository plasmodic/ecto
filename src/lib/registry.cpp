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
      std::cout << "registering ffn for " << name << "\n";
      cellmap[name] = fn;
    }
    
    cell::ptr create(const std::string& name)
    {
      std::map<std::string, ffn_t>::iterator iter = cellmap.find(name);
      if (iter != cellmap.end())
        return iter->second();
      else
        BOOST_THROW_EXCEPTION(EctoException()
                              << diag_msg("Could not find cell")
                              << cell_name(name)); 
    }


  }
}

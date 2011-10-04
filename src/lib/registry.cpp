
#include <map>

#include <ecto/cell.hpp>
#include <ecto/registry.hpp>


namespace ecto {

  namespace registry {

    namespace {
      std::map<std::string, entry_t> cellmap;
    }

    void register_factory_fn(const std::string& name, entry_t e)
    {
      std::cout << "registering ffn for " << name << "\n";
      cellmap[name] = e;
    }
    
    boost::shared_ptr<cell> create(const std::string& name)
    {
      entry_t e = lookup(name);
      return e.construct();
    }

    entry_t lookup(const std::string& name)
    {
      std::map<std::string, entry_t>::iterator iter = cellmap.find(name);
      if (iter != cellmap.end())
        return iter->second;
      else
        BOOST_THROW_EXCEPTION(except::EctoException()
                              << except::diag_msg("Could not find cell")
                              << except::cell_name(name)); 
    }
  }
}

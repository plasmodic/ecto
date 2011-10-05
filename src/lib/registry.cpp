
#include <map>

#include <ecto/cell.hpp>
#include <ecto/registry.hpp>
#include <boost/python.hpp>


namespace ecto {

  namespace registry {

    namespace {
      std::map<std::string, entry_t> cellmap;
    }

    void register_factory_fn(const std::string& name, entry_t e)
    {
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

  namespace py {
    namespace bp = boost::python;

    void postregistration(const std::string& name,
                          const std::string& docstr,
                          const std::string& cpp_typename)
    {
      bp::object thismodule = bp::import("ecto");
      bp::object dict__ = getattr(thismodule, "__dict__");
      bp::object pr = dict__["postregister"];
      pr(name, cpp_typename, docstr, bp::scope());
    }
                   

  }

}

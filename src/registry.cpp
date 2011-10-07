#include <ecto/python.hpp>

namespace ecto {
  namespace py {
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

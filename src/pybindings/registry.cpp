#include <boost/python.hpp>
#include <ecto/ecto.hpp>
#include <ecto/python.hpp>

namespace bp = boost::python;

namespace ecto {
  namespace py {

    void wrap_impl(const std::string& name,
                   const std::string& docstr,
                   const std::string& cpp_typename)
    {
      bp::object thismodule = bp::import("ecto");
      bp::object dict__ = getattr(thismodule, "__dict__");
      bp::object pr = dict__["postregister"];
      pr(name, cpp_typename, docstr, bp::scope());
    }
                   

    void wrapRegistry()
    {
      bp::class_<ecto::registry::entry_t>("registry_entry")
        .def("construct", &ecto::registry::entry_t::construct_)
        .def("declare_params", &ecto::registry::entry_t::declare_params_)
        .def("declare_io", &ecto::registry::entry_t::declare_io_)
        ;

      bp::def("lookup", &ecto::registry::lookup);
    }
  }
}

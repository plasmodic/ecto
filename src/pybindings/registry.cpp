#include <boost/python.hpp>
#include <ecto/ecto.hpp>
#include <ecto/python.hpp>

namespace bp = boost::python;

namespace ecto {
  namespace py {

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

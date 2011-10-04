#include <boost/python.hpp>
#include <ecto/ecto.hpp>
#include <ecto/python.hpp>

namespace bp = boost::python;

namespace ecto {
  namespace py {

    void wrapRegistry()
    {
      bp::def("_create_cell", &ecto::registry::create);
    }
  }
}

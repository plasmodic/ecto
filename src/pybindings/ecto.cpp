#include <boost/python.hpp>
#include <ecto/ecto.hpp>

namespace bp = boost::python;

//forward declare all modules.
namespace ecto {
  namespace py {
    void wrapTendrils();
    void wrapConnection();
    void wrapPlasm();
    void wrapModule();
    void wrapSchedulers();
    void wrapStrand();
    void wrap_except();
  }
}

ECTO_INSTANTIATE_REGISTRY(ecto)
 
BOOST_PYTHON_MODULE(ecto)
{
  bp::class_<ecto::tendril::none>("no_value");

  ecto::py::wrapConnection();
  ecto::py::wrapPlasm();
  ecto::py::wrapModule();
  ecto::py::wrapTendrils();
  ecto::py::wrapSchedulers();
  ecto::py::wrapStrand();
  ecto::py::wrap_except();

  ECTO_REGISTER(ecto);
}


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
  }
}

ECTO_INSTANTIATE_REGISTRY(ecto)
 
BOOST_PYTHON_MODULE(ecto)
{
  ecto::py::wrapConnection();
  ecto::py::wrapPlasm();
  ecto::py::wrapModule();
  ecto::py::wrapTendrils();
  ecto::py::wrapSchedulers();
  ecto::py::wrapStrand();

  ECTO_REGISTER(ecto)
}


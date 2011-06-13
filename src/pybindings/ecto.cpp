#include <boost/python.hpp>

namespace bp = boost::python;

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


BOOST_PYTHON_MODULE(ecto)
{
  //wrap all modules
  ecto::py::wrapConnection();
  ecto::py::wrapPlasm();
  ecto::py::wrapModule();
  ecto::py::wrapTendrils();
  ecto::py::wrapSchedulers();
  ecto::py::wrapStrand();
}


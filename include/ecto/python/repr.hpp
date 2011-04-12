#include <boost/python.hpp>

namespace ecto {
  namespace py {
    inline std::string repr(const boost::python::object& obj)
    {
      return boost::python::extract<std::string>(obj.attr("__repr__")());
    }
  }
}


#include "ecto_split.h"

#include <boost/python.hpp>


namespace bp = boost::python;

BOOST_PYTHON_MODULE(ecto)
{
  //wrap all modules
  ecto::py::wrapConnection();
  ecto::py::wrapPlasm();
  ecto::py::wrapModule();
}




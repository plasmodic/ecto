#pragma once

#include <boost/python/object_fwd.hpp>

namespace ecto {
  namespace py {
    std::string repr(const boost::python::object& obj);
  }
}


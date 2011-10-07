#include <boost/python.hpp>
#include <ecto/python/repr.hpp>
#include <ecto/python/gil.hpp>

namespace ecto {
  namespace py {
    std::string repr(const boost::python::object& obj)
    {
      return boost::python::extract<std::string>(obj.attr("__repr__")());
    }

    struct gil::impl : boost::noncopyable
    {
      PyGILState_STATE gstate;
    };

    gil::gil() : impl_(new gil::impl)
    {
      impl_->gstate = PyGILState_Ensure();
    }
    
    gil::~gil()
    {
      PyGILState_Release(impl_->gstate);
    }
      

  }
}

#include <boost/python.hpp>
#include <ecto/strand.hpp>

namespace bp = boost::python;

namespace ecto
{
  namespace py
  {

    void wrapStrand()
    {
      bp::class_<strand,boost::shared_ptr<strand> >("Strand")
        .add_property("id", &strand::id)
        ;
    }

  }
}


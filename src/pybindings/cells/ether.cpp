#include <ecto/ecto.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/python/overloads.hpp>
namespace bp = boost::python;
namespace ecto
{
  struct EtherSource
  {
    static void
    declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      out.declare<tendril::none>("out", "Any type");
    }
  };

  struct EtherSink
  {
    static void
    declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      in.declare<tendril::none>("in", "Any type");
    }
  };

  bp::tuple
  entangled_pair( tendril::ptr default_value,const std::string& source_name, const std::string& sink_name)
  {
    bp::tuple p;
    cell::ptr source, sink;
    source = ecto::create_cell<EtherSource>();
    source->name(source_name);
    sink = ecto::create_cell<EtherSink>();
    sink->name(sink_name);
    sink->inputs["in"] << default_value;
    source->outputs["out"] = sink->inputs["in"];
    p = bp::make_tuple(source, sink);
    return p;
  }
  BOOST_PYTHON_FUNCTION_OVERLOADS(entangled_pair_overloads, entangled_pair, 1, 3)
  namespace py
  {
    void
    wrap_ether()
    {
      bp::def("EntangledPair", entangled_pair,"Creates an entangled pair of cells.",(bp::arg("default_value"),bp::arg("source_name"),bp::arg("sink_name")));
    }
  }
}

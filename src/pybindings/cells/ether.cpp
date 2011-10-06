#include <ecto/ecto.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/python/overloads.hpp>
namespace bp = boost::python;
namespace ecto
{
  struct EtherSource
  {
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
  entangled_pair(tendril::ptr value,const std::string& source_name="EntagledSource", const std::string& sink_name = "EntagledSink")
  {
    bp::tuple p;
    cell::ptr source, sink;
    source = ecto::create_cell<EtherSource>();
    source->name(source_name);
    sink = ecto::create_cell<EtherSink>();
    sink->name(sink_name);
    sink->inputs["in"] << *value;
    source->outputs.declare("out",sink->inputs["in"]);
    p = bp::make_tuple(source, sink);
    return p;
  }
  BOOST_PYTHON_FUNCTION_OVERLOADS(entangled_pair_overloads, entangled_pair, 1,3)
  namespace py
  {
    using bp::arg;
    void
    wrap_ether()
    {
      bp::def("EntangledPair", entangled_pair,
              entangled_pair_overloads((arg("value"), arg("source_name"), arg("sink_name")), //args
                  "Constructs a pair of entangled cells. Useful for " //
                  "teleportation of tendrils without constructing edges in a graph."//doc str
                  ));
    }
  }
}


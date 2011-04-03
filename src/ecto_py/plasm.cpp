#include <ecto/plasm.hpp>
#include <ecto/module.hpp>
//boost python junk
//#include "ecto_split.h"

#include <boost/python.hpp>
#include <boost/python/args.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

#include "plasm_impl.hpp"
namespace bp = boost::python;
using bp::arg;
namespace ecto
{
  struct plasm_wrapper
  {
    static std::string wrapViz(const ecto::plasm& p)
    {
      return p.viz();
    }

    static bp::dict getModules(plasm& p)
    {
      return p.impl_->modules_.getVerticesPy();
    }

    static bp::list getEdges(plasm& p)
    {
      return p.impl_->modules_.getEdgesPy();
    }

    static void wrap()
    {
      bp::class_<plasm, boost::noncopyable> p("Plasm");
      p.def("connect", &plasm::connect, bp::args("from_module", "output_name", "to_module", "intput_name"));
      p.def("markDirty", &plasm::markDirty);
      p.def("go", &plasm::go);
      p.def("viz", wrapViz, "Get a graphviz string representation of the plasm.");
      p.def("vertices", getModules,
            "Get a dict of the plasm's vertices, with key being integers, and a tuple (module,vertice_type,tendril_key)");
      p.def("edges", getEdges, "Get a list of edges, tuples of two integers(source,target).");

      bp::enum_<plasm::vertex_t> v_enum("vertex_t");
      v_enum.value("root", plasm::root);
      v_enum.value("input", plasm::input);
      v_enum.value("output", plasm::output);
      v_enum.value("param", plasm::param);
    }

  };
  namespace py
  {

    void wrapPlasm()
    {
      plasm_wrapper::wrap();
    }

  }
}


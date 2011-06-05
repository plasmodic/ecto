#include <ecto/plasm.hpp>
#include <ecto/module.hpp>

#include <boost/python.hpp>
#include <boost/python/args.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

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
      throw std::logic_error("not implemented");
      return bp::dict();
    }

    static bp::list getEdges(plasm& p)
    {
      throw std::logic_error("not implemented");
      return bp::list();
    }

    template<typename T>
    static void list_assign(std::list<T>& l, bp::object o)
    {
      // Turn a Python sequence into an STL input range
      bp::stl_input_iterator<T> begin(o), end;
      l.assign(begin, end);
    }

    static void wrap()
    {
      bp::class_<plasm, boost::shared_ptr<plasm>, boost::noncopyable> p("Plasm");
      p.def("insert", &plasm::insert, bp::args("module"),
            "insert module into the graph");
      p.def("connect", &plasm::connect,
            bp::args("from_module", "output_name", "to_module", "intput_name"));
      p.def("disconnect", &plasm::disconnect,
            bp::args("from_module", "output_name", "to_module", "intput_name"));
      /*
      p.def("execute", &plasm::execute,
            "Executes the graph in topological order. Every node will be executed.");
      p.def("spin", &plasm::spin,
            "Causes the graph to execute continuously until finish is called by one of the modules.");
      */
      p.def("viz", wrapViz, "Get a graphviz string representation of the plasm.");
      p.def("vertices", getModules,
            "Get a dict of the plasm's vertices, with key being integers, and a tuple (module,vertice_type,tendril_key)");
      p.def("edges", getEdges,
            "Get a list of edges, tuples of two integers(source,target).");

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


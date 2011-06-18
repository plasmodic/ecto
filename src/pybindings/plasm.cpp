#include <ecto/plasm.hpp>
#include <ecto/module.hpp>

#include <boost/python.hpp>
#include <boost/python/raw_function.hpp>
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

    static void plasm_connect_list(plasm& p, bp::list connections)
    {
      bp::stl_input_iterator<bp::tuple> begin(connections), end;
      while (begin != end)
      {
        bp::tuple x = *(begin++);
        module::ptr from = bp::extract<module::ptr>(x[0]);
        module::ptr to = bp::extract<module::ptr>(x[2]);
        std::string output = bp::extract<std::string>(x[1]), input =
            bp::extract<std::string>(x[3]);
        p.connect(from, output, to, input);
      }
    }
    static int plasm_connect_args(boost::python::tuple args, bp::dict kw)
    {
      int i = 0;
      plasm::ptr p = bp::extract<plasm::ptr>(args[i++]);
      for (int end = bp::len(args); i < end; i++)
      {
        plasm_connect_list(*p, bp::list(args[i]));
      }
      return i;
    }
    static bp::list plasm_get_connections(plasm& p)
    {
      bp::list result;
      const ecto::graph::graph_t& g = p.graph();
      ecto::graph::graph_t::edge_iterator begin, end;
      ecto::graph::graph_t::vertex_descriptor source, sink;
      for (boost::tie(begin, end) = boost::edges(g); begin != end; ++begin)
      {
        source = boost::source(*begin, g);
        sink = boost::target(*begin, g);
        module::ptr to = g[sink], from = g[source];
        std::string to_port = g[*begin]->to_port;
        std::string from_port = g[*begin]->from_port;
        result.append(bp::make_tuple(from, from_port, to, to_port));
      }
      return result;
    }
    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS( execute_overloads , plasm::execute , 0,1)
    static void wrap()
    {
      using bp::arg;

      bp::class_<plasm, boost::shared_ptr<plasm>, boost::noncopyable> p(
          "Plasm");
      p.def("insert", &plasm::insert, bp::args("module"),
            "insert module into the graph");

      p.def("connect", &plasm_connect_list, bp::args("connection_list"));
      p.def("connect", bp::raw_function(plasm_connect_args, 2));
      p.def("disconnect", &plasm::disconnect,
            bp::args("from_module", "output_name", "to_module", "intput_name"));
      p.def("connect", &plasm::connect,
            bp::args("from_module", "output_name", "to_module", "intput_name"));
      p.def(
          "execute",
          &plasm::execute,
          execute_overloads(
              bp::args("niter"),
              "Executes the graph in topological order. Every node will be executed."));

      p.def("viz", wrapViz,
            "Get a graphviz string representation of the plasm.");
      p.def(
          "connections",
          plasm_get_connections,
          "Grabs a the current list based description of the graph. "
          "Its a list of tuples (from_module, output_key, to_module, input_key)");

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


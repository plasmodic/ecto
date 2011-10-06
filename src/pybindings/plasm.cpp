#include <ecto/plasm.hpp>
#include <ecto/cell.hpp>


#include <boost/python.hpp>
#include <boost/python/raw_function.hpp>
#include <boost/python/args.hpp>
#include <boost/python/type_id.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <ecto/ecto.hpp>
#include <ecto/serialization/registry.hpp>
#include <ecto/serialization/cell.hpp>

namespace bp = boost::python;
using bp::arg;

namespace ecto
{
  namespace plasm_wrapper
  {
    std::string wrapViz(const ecto::plasm& p)
    {
      return p.viz();
    }

    template<typename T>
    void list_assign(std::list<T>& l, bp::object o)
    {
      // Turn a Python sequence into an STL input range
      bp::stl_input_iterator<T> begin(o), end;
      l.assign(begin, end);
    }

    bp::list sanitize_connection_list(bp::list connections)
    {
      int len = bp::len(connections);
      bp::list tuples;
      for (int i = 0; i < len; ++i)
      {
        bp::extract<bp::tuple> te(connections[i]);
        bp::extract<bp::list> le(connections[i]);
        if (te.check())
        {
          tuples.append(te());
        }
        else if (le.check())
        {
          tuples += le();
        }
        else
        {
          throw std::runtime_error(
              "Expecting the connection list to contain only lists of tuples, or tuples, no other types.");
        }
      }
      return tuples;
    }

    void plasm_connect_explicit(plasm& p, 
                                bp::object fromcell, std::string fromport,
                                bp::object tocell, std::string toport)
    {
      bp::object fc_impl = getattr(fromcell, "__impl");
      cell::ptr fc = bp::extract<cell::ptr>(fc_impl);
      bp::object tc_impl = getattr(tocell, "__impl");
      cell::ptr tc = bp::extract<cell::ptr>(tc_impl);
      p.connect(fc, fromport, tc, toport);
    }
                                
    void plasm_disconnect_explicit(plasm& p, 
                                   bp::object fromcell, std::string fromport,
                                   bp::object tocell, std::string toport)
    {
      bp::object fc_impl = getattr(fromcell, "__impl");
      cell::ptr fc = bp::extract<cell::ptr>(fc_impl);
      bp::object tc_impl = getattr(tocell, "__impl");
      cell::ptr tc = bp::extract<cell::ptr>(tc_impl);
      p.disconnect(fc, fromport, tc, toport);
    }
                                

    void plasm_connect_list(plasm& p, bp::list connections)
    {
      connections = sanitize_connection_list(connections);
      bp::stl_input_iterator<bp::tuple> begin(connections), end;
      while (begin != end)
      {
        bp::tuple x = *(begin++);
        cell::ptr from = bp::extract<cell::ptr>(x[0]);
        cell::ptr to = bp::extract<cell::ptr>(x[2]);
        std::string output = bp::extract<std::string>(x[1]), input = bp::extract<std::string>(x[3]);
        p.connect(from, output, to, input);
      }
    }

    int plasm_connect_args(boost::python::tuple args, bp::dict kw)
    {
      int i = 0;
      plasm::ptr p = bp::extract<plasm::ptr>(args[i++]);
      for (int end = bp::len(args); i < end; i++)
      {
        bp::list l;
        try
          {
          l = bp::list(args[i]);
          } catch (const boost::python::error_already_set&)
        {
          PyErr_Clear(); //Need to clear the error or python craps out. Try commenting out and running the doc tests.
          throw std::runtime_error(
              "Did you mean plasm.connect(cellA['out'] >> cellB['in']), or plasm.connect(cellA,'out',cellB,'in')?");
              }
        plasm_connect_list(*p, l);
      }
      return i;
    }

    void
    plasm_save(plasm&p,std::string filename)
    {
      std::ofstream out(filename.c_str());
      p.save(out);
    }

    void
    plasm_load(plasm&p,std::string filename)
    {
      std::ifstream in(filename.c_str());
      p.load(in);
    }

    bp::list plasm_get_connections(plasm& p)
    {
      bp::list result;
      const ecto::graph::graph_t& g = p.graph();
      ecto::graph::graph_t::edge_iterator begin, end;
      ecto::graph::graph_t::vertex_descriptor source, sink;
      for (boost::tie(begin, end) = boost::edges(g); begin != end; ++begin)
      {
        source = boost::source(*begin, g);
        sink = boost::target(*begin, g);
        cell::ptr to = g[sink], from = g[source];
        std::string to_port = g[*begin]->to_port;
        std::string from_port = g[*begin]->from_port;
        result.append(bp::make_tuple(from, from_port, to, to_port));
      }
      return result;
    }

    struct bplistappender
    {
      bplistappender(bp::list&l) : l(l) { }

      void operator()(ecto::cell::ptr c) { l.append(c); }
      bp::list& l;
    };

    bp::list plasm_get_cells(plasm& p)
    {
      bp::list l;
      std::vector<cell::ptr> cells = p.cells();
      std::for_each(cells.begin(),cells.end(),bplistappender(l));
      return l;
    }

    void plasm_insert(plasm& p, bp::object bb)
    {
      bp::object cbp = bb.attr("__impl");
      cell::ptr c =  bp::extract<cell::ptr>(cbp);
      p.insert(c);
    }
    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS( execute_overloads , plasm::execute , 0,1)

    void wrap()
    {
      using bp::arg;

      bp::class_<plasm, boost::shared_ptr<plasm>, boost::noncopyable> p("Plasm");
      p.def("insert", &plasm_insert, bp::args("cell"), "insert a black box into the graph");
      p.def("insert", &plasm::insert, bp::args("cell"), "insert cell into the graph");//order is important here.

      p.def("connect", &plasm_connect_list, bp::args("connection_list"));
      p.def("connect", bp::raw_function(plasm_connect_args, 2));
      p.def("connect", &plasm_connect_explicit, 
            bp::args("from_cell", "output_name", "to_cell", "intput_name"));
      p.def("disconnect", &plasm_disconnect_explicit, 
            bp::args("from_cell", "output_name", "to_cell", "intput_name"));
      p.def("execute", &plasm::execute,
            execute_overloads(bp::args("niter"),
                              "Executes the graph in topological order. Every node will be executed."));

      p.def("viz", wrapViz, "Get a graphviz string representation of the plasm.");
      p.def("connections", plasm_get_connections, "Grabs the current list based description of the graph. "
            "Its a list of tuples (from_cell, output_key, to_cell, input_key)");
      p.def("cells", plasm_get_cells, "Grabs the current set of cells that are in the plasm.");
      p.def("check", &plasm::check);
      p.def("configure_all", &plasm::configure_all);
      p.def("save",plasm_save);
      p.def("load",plasm_load);

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


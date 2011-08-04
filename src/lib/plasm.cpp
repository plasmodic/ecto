#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/cell.hpp>
#include <ecto/graph_types.hpp>
#include <ecto/schedulers/singlethreaded.hpp>

#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/regex.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

namespace ecto
{
  namespace
  {
    using boost::tie;
    using boost::add_vertex;

    graph::edge::ptr
    make_edge(const std::string& fromport, const std::string& toport)
    {
      graph::edge::ptr eptr(new graph::edge(fromport, toport));
      return eptr;
    }
  } // namespace

  using namespace graph;
  #define STRINGY_DINGY(A) #A
  //see http://www.graphviz.org/content/node-shapes for reference.
  const char* table_str = STRINGY_DINGY(
      <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4">
      %s
      <TR>
      %s
      %s
      </TR>
      %s
      %s
      </TABLE>
  );

  const char* input_str = STRINGY_DINGY(
      <TD PORT="i_%s" BGCOLOR="springgreen">%s</TD>
  );

  const char* cell_str = 
    STRINGY_DINGY(<TD ROWSPAN="%d" COLSPAN="%d" BGCOLOR="khaki"> %s </TD>
  );

  const char* param_str_1st = STRINGY_DINGY(
      <TD PORT="p_%s" BGCOLOR="lightblue">%s</TD>
  );

  const char* param_str_N = STRINGY_DINGY(
      <TR>
      <TD PORT="p_%s" BGCOLOR="lightblue">%s</TD>
      </TR>
  );

  const char* output_str = STRINGY_DINGY(
      <TD PORT="o_%s" BGCOLOR="indianred1">%s</TD>
  );

  struct vertex_writer
  {
    graph_t* g;

    vertex_writer(graph_t* g_)
        :
          g(g_)
    {
    }

    void
    operator()(std::ostream& out, graph_t::vertex_descriptor vd)
    {


      cell::ptr c = (*g)[vd];
      int n_inputs = c->inputs.size();
      int n_outputs = c->outputs.size();
      int n_params = c->parameters.size();
      std::string sanitized_name = c->name();
      //deal with html characters: ", &, <, and >
      const boost::regex esc_lt("[<]");
      const std::string rep_lt("&lt;");
      const boost::regex esc_gt("[>]");
      const std::string rep_gt("&gt;");

      sanitized_name = boost::regex_replace(sanitized_name, esc_lt, rep_lt, boost::match_default);
      sanitized_name = boost::regex_replace(sanitized_name, esc_gt, rep_gt, boost::match_default);

      std::string inputs;
      BOOST_FOREACH(const tendrils::value_type& x, c->inputs)
          {
            std::string key = x.first;
            if (inputs.empty())
              inputs = "<TR>\n";
            inputs += boost::str(boost::format(input_str) % key % key) + "\n";
          }
      if (!inputs.empty())
        inputs += "</TR>";

      std::string outputs;
      BOOST_FOREACH(const tendrils::value_type& x, c->outputs)
          {
            std::string key = x.first;
            if (outputs.empty())
              outputs = "<TR>\n";
            outputs += boost::str(boost::format(output_str) % key % key) + "\n";
          }
      if (!outputs.empty())
        outputs += "</TR>";

      std::string cellrow = boost::str(
          boost::format(cell_str) % std::max(1,n_params) % int(std::max(1,std::max(n_inputs, n_outputs))) % sanitized_name);
      std::string p1, pN;
      BOOST_FOREACH(const tendrils::value_type& x, c->parameters)
          {
            std::string key = x.first;
            if (p1.empty())
              p1 = boost::str(boost::format(param_str_1st) % key % key) + "\n";
            else
              pN += boost::str(boost::format(param_str_N) % key % key) + "\n";
          }

      std::string table = boost::str(boost::format(table_str) % inputs % cellrow % p1 % pN % outputs);
      out << "[label=<" << table << ">]";
    }
  };

  struct edge_writer
  {
    graph_t* g;

    edge_writer(graph_t* g_)
        :
          g(g_)
    {
    }

    void
    operator()(std::ostream& out, graph_t::edge_descriptor ed)
    {
      out << "[headport=\"i_" << (*g)[ed]->to_port << "\" tailport=\"o_" << (*g)[ed]->from_port << "\"]";
    }
  };

  struct graph_writer
  {
    void
    operator()(std::ostream& out) const
    {
      out << "graph [rankdir=TB, ranksep=1]" << std::endl;
      out << "edge [labelfontsize=8]" << std::endl;
      out << "node [shape=plaintext]" << std::endl;
    }
  };

  struct plasm::impl
  {
    impl()
    {
    }

    //insert a cell into the graph, will retrieve the
    //vertex descriptor if its already in the graph...
    graph_t::vertex_descriptor
    insert_module(cell::ptr m)
    {
      //use the vertex map to look up the graphviz descriptor (reverse lookup)
      ModuleVertexMap::iterator it = mv_map.find(m);
      if (it != mv_map.end())
        return it->second;
      graph_t::vertex_descriptor d = add_vertex(m, graph);
      mv_map.insert(std::make_pair(m, d));
      return d;
    }

    void
    connect(cell::ptr from, std::string output, cell::ptr to, std::string input)
    {
      //connect does all sorts of type checking so that connections are always valid.
      tendril::ptr from_port, to_port;
      try
      {
        from_port = from->outputs[output];
      } catch (ecto::except::EctoException& e)
      {
        e << boost::str(boost::format("'%s.outputs.%s' does not exist.") % from->name() % output);
        throw;
      }
      try
      {
        to_port = to->inputs[input];
      } catch (ecto::except::EctoException& e)
      {
        e << boost::str(boost::format("'%s.inputs.%s' does not exist.") % from->name() % output);
        throw;
      }
      //throw if the types are bad... Don't allow erroneous graph construction
      //also this is more local to the error.
      if (!to_port->compatible_type(*from_port))
      {
        std::string s;
        s = boost::str(boost::format("type mismatch:  '%s.outputs.%s' of type '%s' is connected to"
                                     "'%s.inputs.%s' of type '%s'")
                       % from->name()
                       % output
                       % from_port->type_name()
                       % to->name()
                       % input
                       % to_port->type_name());
        throw ecto::except::TypeMismatch(s);
      }

      graph_t::vertex_descriptor fromv = insert_module(from), tov = insert_module(to);
      edge::ptr new_edge = make_edge(output, input);

      //assert that the new edge does not violate inputs that are already connected.
      //RULE an input may only have one source.
      graph_t::in_edge_iterator inbegin, inend;
      tie(inbegin, inend) = boost::in_edges(tov, graph);
      while (inbegin != inend)
      {
        edge::ptr e = graph[*inbegin];
        if (e->to_port == new_edge->to_port)
        {
          std::string s;
          s = boost::str(
              boost::format("graph error: '%s.inputs.%s' is already connected, this is considered an error!")
              % to->name()
              % to);
          throw except::EctoException(s);
        }
        ++inbegin;
      }

      bool added;
      graph_t::edge_descriptor ed;
      tie(ed, added) = boost::add_edge(fromv, tov, new_edge, graph);
      if (!added)
      {
        throw std::runtime_error(
            "failed to connect " + from->name() + ":" + output + " with " + to->name() + ":" + input);
      }
    }

    void
    disconnect(cell::ptr from, std::string output, cell::ptr to, std::string input)
    {
      graph_t::vertex_descriptor fromv = insert_module(from), tov = insert_module(to);
      boost::remove_edge(fromv, tov, graph);
    }

    //the cell to vertex mapping
    //unordered_map so that cell ptr works as a key...
    typedef boost::unordered_map<ecto::cell::ptr, graph_t::vertex_descriptor> ModuleVertexMap;
    ModuleVertexMap mv_map;
    graph_t graph;
    boost::shared_ptr<ecto::schedulers::singlethreaded> scheduler;
    struct CVMtoCell
    {
      cell::ptr
      operator()(const ModuleVertexMap::value_type& v)
      {
        return v.first;
      }
    };
  };

  plasm::plasm() : impl_(new impl) { }


  plasm::~plasm() { }

  void
  plasm::insert(cell::ptr mod)
  {
    impl_->insert_module(mod);
  }

  void
  plasm::connect(cell::ptr from, const std::string& output, cell::ptr to, const std::string& input)
  {
    impl_->connect(from, output, to, input);
  }


  void
  plasm::viz(std::ostream& out) const
  {
    boost::write_graphviz(out, impl_->graph, vertex_writer(&impl_->graph), edge_writer(&impl_->graph), graph_writer());
  }

  std::string
  plasm::viz() const
  {
    std::stringstream ss;
    viz(ss);
    return ss.str();
  }

  void
  plasm::disconnect(cell_ptr from, const std::string& output, cell_ptr to, const std::string& input)
  {
    impl_->disconnect(from, output, to, input);
  }

  graph::graph_t&
  plasm::graph()
  {
    return impl_->graph;
  }

  int
  plasm::execute(unsigned niter)
  {
    if (!impl_->scheduler)
      impl_->scheduler.reset(new ecto::schedulers::singlethreaded(shared_from_this()));
    impl_->scheduler->execute(niter);
    while (impl_->scheduler->running())
      boost::this_thread::sleep(boost::posix_time::microseconds(10));

    return 0;
  }

  std::size_t 
  plasm::size() const
  {
    return num_vertices(impl_->graph);
  }

  std::vector<cell::ptr>
  plasm::cells() const
  {
    std::vector<cell::ptr> c;
    std::transform(impl_->mv_map.begin(), impl_->mv_map.end(), std::back_inserter(c), impl::CVMtoCell());
    return c;
  }
  void plasm::configure_all()
  {
    BOOST_FOREACH(impl::ModuleVertexMap::value_type& x, impl_->mv_map)
    {
      x.first->configure();
    }
  }

  void
  plasm::check() const
  {
    graph_t& g(impl_->graph);
    graph_t::vertex_iterator begin, end;
    tie(begin, end) = boost::vertices(g);
    while (begin != end)
    {
      cell::ptr m = g[*begin];
      std::set<std::string> in_connected, out_connected;

      //verify all required inputs are connected
      graph_t::in_edge_iterator b_in, e_in;
      tie(b_in, e_in) = boost::in_edges(*begin, g);
      while (b_in != e_in)
      {
        edge::ptr in_edge = g[*b_in];
        cell::ptr from_module = g[source(*b_in, g)];
        in_connected.insert(in_edge->to_port);
        ++b_in;
      }

      for (tendrils::const_iterator b_tend = m->inputs.begin(), e_tend = m->inputs.end(); b_tend != e_tend; ++b_tend)
      {

        if (b_tend->second->required() && in_connected.count(b_tend->first) == 0)
        {
          std::string s = str(boost::format("in module %s, input port '%s' is required"
                                            " but not connected")
                              % m->name()
                              % b_tend->first);
          throw except::EctoException(s);
        }
      }

      //verify the outputs are connected
      graph_t::out_edge_iterator b_out, e_out;
      tie(b_out, e_out) = boost::out_edges(*begin, g);
      while (b_out != e_out)
      {
        edge::ptr out_edge = g[*b_out];
        out_connected.insert(out_edge->from_port);
        ++b_out;
      }

      for (tendrils::const_iterator b_tend = m->outputs.begin(), e_tend = m->outputs.end(); b_tend != e_tend; ++b_tend)
      {
        if (b_tend->second->required() && out_connected.count(b_tend->first) == 0)
        {
          std::string s = str(boost::format("in module %s, output port '%s' is required"
                                            " but not connected")
                              % m->name()
                              % b_tend->first);
          throw except::EctoException(s);
        }
      }

      ++begin;
    }
  }

}

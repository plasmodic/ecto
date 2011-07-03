#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/module.hpp>
#include <ecto/graph_types.hpp>
#include <ecto/scheduler/singlethreaded.hpp>

#include <boost/format.hpp>

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

  struct plasm::impl
  {
    impl()
    {
    }

    //insert a module into the graph, will retrieve the
    //vertex descriptor if its already in the graph...
    graph_t::vertex_descriptor
    insert_module(module::ptr m)
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
    connect(module::ptr from, std::string output, module::ptr to, std::string input)
    {
      //connect does all sorts of type checking so that connections are always valid.
      tendril::ptr from_port, to_port;
      try
      {
        from_port = from->outputs.at(output);
      } catch (ecto::except::EctoException& e)
      {
        e << boost::str(boost::format("'%s.outputs.%s' does not exist.") % from->name() % output);
        throw;
      }
      try
      {
        to_port = to->inputs.at(input);
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
    disconnect(module::ptr from, std::string output, module::ptr to, std::string input)
    {
      graph_t::vertex_descriptor fromv = insert_module(from), tov = insert_module(to);
      boost::remove_edge(fromv, tov, graph);
    }

    //the module to vertex mapping
    //unordered_map so that module ptr works as a key...
    typedef boost::unordered_map<ecto::module::ptr, graph_t::vertex_descriptor> ModuleVertexMap;
    ModuleVertexMap mv_map;
    graph_t graph;
    boost::shared_ptr<ecto::scheduler::singlethreaded> scheduler;
  };

  plasm::plasm()
      :
        impl_(new impl)
  {
    impl_->scheduler.reset(new ecto::scheduler::singlethreaded(*this));
  }

  plasm::~plasm()
  {
  }

  void
  plasm::insert(module::ptr mod)
  {
    impl_->insert_module(mod);
  }

  void
  plasm::connect(module::ptr from, const std::string& output, module::ptr to, const std::string& input)
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
  plasm::disconnect(module_ptr from, const std::string& output, module_ptr to, const std::string& input)
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
    return impl_->scheduler->execute(niter);
  }

  void
  plasm::check() const
  {
    graph_t& g(impl_->graph);
    graph_t::vertex_iterator begin, end;
    tie(begin, end) = boost::vertices(g);
    while (begin != end)
    {
      module::ptr m = g[*begin];
      std::set<std::string> in_connected, out_connected;

      //verify all required inputs are connected
      graph_t::in_edge_iterator b_in, e_in;
      tie(b_in, e_in) = boost::in_edges(*begin, g);
      while (b_in != e_in)
      {
        edge::ptr in_edge = g[*b_in];
        module::ptr from_module = g[source(*b_in, g)];
        in_connected.insert(in_edge->to_port);
        ++b_in;
      }

      for (tendrils::const_iterator b_tend = m->inputs.begin(), e_tend = m->inputs.end(); b_tend != e_tend; ++b_tend)
      {

        if (b_tend->second->required() and in_connected.count(b_tend->first) == 0)
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
        if (b_tend->second->required() and out_connected.count(b_tend->first) == 0)
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

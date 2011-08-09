#include "plasm/impl.hpp"

namespace ecto {

  using graph::graph_t;
  using graph::edge;

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


  plasm::impl::impl() {}

    //insert a cell into the graph, will retrieve the
    //vertex descriptor if its already in the graph...
  graph_t::vertex_descriptor
  plasm::impl::insert_module(cell::ptr m)
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
  plasm::impl::connect(cell::ptr from, std::string output, cell::ptr to, std::string input)
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
  plasm::impl::disconnect(cell::ptr from, std::string output, cell::ptr to, std::string input)
  {
    graph_t::vertex_descriptor fromv = insert_module(from), tov = insert_module(to);
    boost::remove_edge(fromv, tov, graph);
  }

}

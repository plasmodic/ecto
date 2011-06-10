#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/module.hpp>
#include <ecto/graph_types.hpp>
#include <ecto/scheduler/singlethreaded.hpp>

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

    graph::edge::ptr make_edge(const std::string& fromport,
                               const std::string& toport)
    {
      graph::edge::ptr eptr(new graph::edge(fromport, toport));
      return eptr;
    }

  } // namespace

  using namespace graph;

  struct plasm::impl
  {
    impl() { }

    //insert a module into the graph, will retrieve the
    //vertex descriptor if its already in the graph...
    graph_t::vertex_descriptor insert_module(module::ptr m)
    {
      //use the vertex map to look up the graphviz descriptor (reverse lookup)
      ModuleVertexMap::iterator it = mv_map.find(m);
      if (it != mv_map.end())
        return it->second;
      graph_t::vertex_descriptor d = add_vertex(m, graph);
      mv_map.insert(std::make_pair(m, d));
      return d;
    }

    void connect(module::ptr from, std::string output, module::ptr to,
                 std::string input)
    {
      //throw if the types are bad...
      to->inputs[input].enforce_compatible_type(from->outputs[output]);

      graph_t::vertex_descriptor fromv = insert_module(from), tov =
          insert_module(to);
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
              throw std::runtime_error(new_edge->to_port
                                       + " is already connected, this is considered an error");
            }
          ++inbegin;
        }

      bool added;
      graph_t::edge_descriptor ed;
      tie(ed, added) = boost::add_edge(fromv, tov, new_edge, graph);
      if (!added)
        {
          throw std::runtime_error("failed to connect " + from->name() + ":"
                                   + output + " with " + to->name() + ":"
                                   + input);
        }
    }

    void disconnect(module::ptr from, std::string output, module::ptr to,
                    std::string input)
    {
      graph_t::vertex_descriptor 
        fromv = insert_module(from), tov = insert_module(to);
      boost::remove_edge(fromv, tov, graph);
    }

    //the module to vertex mapping
    //unordered_map so that module ptr works as a key...
    typedef boost::unordered_map<ecto::module::ptr, graph_t::vertex_descriptor>
        ModuleVertexMap;
    ModuleVertexMap mv_map;
    graph_t graph;
    boost::shared_ptr<ecto::scheduler::singlethreaded> scheduler;
  };

  plasm::plasm() :
    impl_(new impl)
  {
    impl_->scheduler.reset(new ecto::scheduler::singlethreaded(*this));
  }

  void plasm::insert(module::ptr mod)
  {
    impl_->insert_module(mod);
  }

  void plasm::connect(module::ptr from, const std::string& output,
                      module::ptr to, const std::string& input)
  {
    impl_->connect(from, output, to, input);
  }

  void plasm::viz(std::ostream& out) const
  {
    boost::write_graphviz(out, impl_->graph, vertex_writer(&impl_->graph),
                          edge_writer(&impl_->graph), graph_writer());
  }

  std::string plasm::viz() const
  {
    std::stringstream ss;
    viz(ss);
    return ss.str();
  }

  void plasm::disconnect(module_ptr from, const std::string& output,
                         module_ptr to, const std::string& input)
  {
    impl_->disconnect(from, output, to, input);
  }

  graph::graph_t& plasm::graph()
  {
    return impl_->graph;
  }
  
  int plasm::execute()
  {
    return impl_->scheduler->execute();
  }
}

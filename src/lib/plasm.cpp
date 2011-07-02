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
      // to->inputs.at(input)->enforce_compatible_type(*from->outputs.at(output));

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

  plasm::~plasm()
  {
    //call destroy on all modules.
//    plasm::impl::ModuleVertexMap::iterator it = impl_->mv_map.begin();
//    while (it != impl_->mv_map.end())
//    {
//      it->first->destroy();
//      ++it;
//    }
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
  
  int plasm::execute(unsigned niter)
  {
    return impl_->scheduler->execute(niter);
  }

  void plasm::check() const
  {
    graph_t& g(impl_->graph);
    graph_t::vertex_iterator begin, end;
    tie(begin, end) = boost::vertices(g);
    while(begin != end)
      {
        module::ptr m = g[*begin];
        assert(m && "uh oh, there is no module there...");
        std::cout << m->name() << "\n";

        std::set<std::string> in_connected, out_connected;


        graph_t::in_edge_iterator b_in, e_in;
        tie(b_in, e_in) = boost::in_edges(*begin, g);
        while(b_in != e_in)
          {
            edge::ptr in_edge = g[*b_in];

            module::ptr from_module = g[source(*b_in, g)];
            std::cout << " in: " << in_edge->to_port << "\n";
            tendril::const_ptr from_tendril = from_module->outputs.at(in_edge->from_port);
            tendril::const_ptr to_tendril = m->inputs.at(in_edge->to_port);

            if (! from_tendril->compatible_type(*to_tendril))
              {
                std::string s;
                s = str(boost::format("type mismatch:  '%s.outputs.%s' of type '%s' is connected to"
                                      "'%s.inputs.%s' of type '%s'")
                        % from_module->name() % in_edge->from_port % from_tendril->type_name()
                        % m->name() % in_edge->to_port % to_tendril->type_name());
                throw std::runtime_error(s);
              }
            in_connected.insert(in_edge->to_port);
            ++b_in;
          }

        for (tendrils::const_iterator b_tend = m->inputs.begin(),
               e_tend = m->inputs.end();
             b_tend != e_tend;
             ++b_tend)
          {
            if (b_tend->second->required() and  in_connected.count(b_tend->first) == 0)
              throw std::runtime_error(str(boost::format("in module %s, input port '%s' is required"
                                                         " but not connected")
                                           % m->name() % b_tend->first));
          }

        graph_t::out_edge_iterator b_out, e_out;
        tie(b_out, e_out) = boost::out_edges(*begin, g);
        while(b_out != e_out)
          {
            edge::ptr out_edge = g[*b_out];
            // code for extra typechecking... currently useless as typechecking is done at connect time.
            // but maybe we want to postpone our typechecks until execute() time.

            module::ptr to_module = g[target(*b_out, g)];
            std::cout << " out: " << out_edge->from_port << "\n";
            tendril::const_ptr from_tendril = m->outputs.at(out_edge->from_port);
            tendril::const_ptr to_tendril = to_module->inputs.at(out_edge->to_port);

            if (! from_tendril->compatible_type(*to_tendril))
              {
                std::string s;
                s = str(boost::format("type mismatch:  '%s.outputs.%s' of type '%s' is connected to"
                                      "'%s.inputs.%s' of type '%s'")
                        % m->name() % out_edge->from_port % from_tendril->type_name()
                        % to_module->name() % out_edge->to_port % to_tendril->type_name());
                throw std::runtime_error(s);
              }

            out_connected.insert(out_edge->from_port);
            ++b_out;
          }

        for (tendrils::const_iterator b_tend = m->outputs.begin(),
               e_tend = m->outputs.end();
             b_tend != e_tend;
             ++b_tend)
          {
            if (b_tend->second->required() and  out_connected.count(b_tend->first) == 0)
              throw std::runtime_error(str(boost::format("in module %s, output port '%s' is required"
                                                         " but not connected")
                                           % m->name() % b_tend->first));
          }




        ++begin;
      }
  }

}

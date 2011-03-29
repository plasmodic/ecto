#include <ecto/plasm2.hpp>
#include <ecto/tendril.hpp>
#include <ecto/module.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>

//this quiets a deprecated warning
#define BOOST_NO_HASH
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/graph/graphviz.hpp>

using boost::vertex_property_tag;
using boost::adjacency_list;
using boost::vecS;
using boost::bidirectionalS;
using boost::property;
using boost::property_map;
using boost::graph_traits;
namespace ecto
{

  struct ModuleGraph
  {
    typedef std::pair<module::ptr, std::string> _Vertex;
    struct Vertex : public _Vertex
    {
      Vertex() :
        _Vertex()
      {
      }
      Vertex(const module::ptr& p, const std::string& s) :
        _Vertex(p, s)
      {
      }

      struct Tag
      {
        typedef vertex_property_tag kind;
      };
      typedef property<Tag, Vertex> Property;
      size_t uid;
    };
    struct opless{
    inline bool operator()(const Vertex&lhs, const Vertex& rhs) const
    {
      return lhs.first.get() < rhs.first.get() ||  (lhs.first.get() == rhs.first.get()  && lhs.second < rhs.second);
    }
    };

    typedef adjacency_list<boost::setS, vecS, boost::bidirectionalS, Vertex::Property> graph_t;
    typedef graph_traits<graph_t> GraphTraits;
    typedef graph_traits<graph_t>::vertex_descriptor Vertex_Desc;

    graph_t graph_, root_graph_;

    void add_edge(module::ptr from_m, const std::string& out_name, module::ptr to_m, const std::string& in_name)
    {
      Vertex from = make_vert(from_m, out_name);
      Vertex to = make_vert(to_m, in_name);
      Vertex root_from = make_vert(from_m);
      Vertex root_to = make_vert(to_m);
      boost::add_edge(root_from.uid, from.uid, graph_);
      boost::add_edge(root_from.uid, root_to.uid, root_graph_);
      boost::add_edge(from.uid, to.uid, graph_);
      boost::add_edge(to.uid, root_to.uid, graph_);
    }

    Vertex& get_vert(size_t uid)
    {
      return boost::get(Vertex::Tag(), graph_)[uid];
    }

    Vertex make_vert(const module::ptr& p, const std::string& s = "root")
    {
      Vertex v(p, s);
      getUID(v);
      get_vert(v.uid) = v;
      return v;
    }

    struct Dirtier
    {
      Dirtier(ModuleGraph&g) :
        graph(g)
      {
      }

      void operator()(const Vertex_Desc& v)
      {
        graph.get_vert(v).first->dirty(true);
        GraphTraits::out_edge_iterator out_i, out_end;
        GraphTraits::edge_descriptor e;
        for (boost::tie(out_i, out_end) = boost::out_edges(v, graph.root_graph_); out_i != out_end; ++out_i)
        {
          e = *out_i;
          Vertex_Desc targ = target(e, graph.root_graph_);
          (*this)(targ);
        }
      }
      ModuleGraph& graph;
    };

    struct Goer
    {
      Goer(ModuleGraph&g) :
        graph(g)
      {
      }
      void operator()(const Vertex_Desc& v)
      {

        Vertex& vert = graph.get_vert(v);
        if (vert.first->dirty())
          return;
        GraphTraits::in_edge_iterator in_i, in_end;
        GraphTraits::edge_descriptor e;
        for (boost::tie(in_i, in_end) = boost::in_edges(v, graph.root_graph_); in_i != in_end; ++in_i)
        {
          e = *in_i;
          Vertex_Desc targ = boost::source(e, graph.root_graph_);
          (*this)(targ);
        }
        vert.first->Process();
        vert.first->dirty(true);
      }
      ModuleGraph& graph;
    };

    void mark_dirty(module::ptr m)
    {
      Dirtier(*this)(make_vert(m).uid);
    }
    void go(module::ptr m)

    {
      Goer(*this)(make_vert(m).uid);
    }

    typedef std::set<Vertex,opless> module_set_t;
     module_set_t module_set;

    /** Assigns a unique vertex id, from the graph.
     *
     * @param v
     */
    inline void getUID(Vertex& v)
    {
      //std::cout << v.first << v.second << std::endl;
      module_set_t::const_iterator it = module_set.find(v);
      if (it == module_set.end())
      {
        v.uid = boost::add_vertex(graph_);
        module_set.insert(v).second;
      }
      else
        v.uid = it->uid;

    }

  };

  struct plasm2::impl
  {

    ModuleGraph modules_;
  };

  plasm2::plasm2() :
    impl_(new impl)
  {
  }
  void plasm2::connect(module::ptr from, const std::string& out_name, module::ptr to, const std::string& in_name)
  {
    impl_->modules_.add_edge(from, out_name, to, in_name);
    from->connect(out_name, to, in_name);
  }

  void plasm2::markDirty(module::ptr m)
  {
    // Access the property accessor type for this graph
    impl_->modules_.mark_dirty(m);

  }
  void plasm2::go(module::ptr m)
  {
    impl_->modules_.go(m);
  }

  void plasm2::viz(std::ostream& out) const
  {
    boost::write_graphviz(out, impl_->modules_.graph_);
  }

  std::string plasm2::viz() const
  {
    std::stringstream ss;
    viz(ss);
    return ss.str();
  }

}

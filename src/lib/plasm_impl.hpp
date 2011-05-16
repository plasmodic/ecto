#pragma once
#include <ecto/plasm.hpp>
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

#include <boost/foreach.hpp>

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

  struct Vertex: public _Vertex
  {
    Vertex() :
      _Vertex()
    {
    }

    Vertex(const module::ptr& p, const std::string& s, plasm::vertex_t t) :
      _Vertex(p, s), uid(-1), ecto_type(t)
    {

    }

    struct Tag
    {
      typedef vertex_property_tag kind;
    };
    typedef property<Tag, Vertex> Property;
    size_t uid;
    plasm::vertex_t ecto_type;
    tendril getTendril()
    {
      switch (ecto_type)
      {
      case plasm::root:
        return tendril();
        break;
      case plasm::input:
        return first->inputs.at(second);
        break;
      case plasm::output:
        return first->outputs.at(second);
        break;
      case plasm::param:
        return first->params.at(second);
        break;
      }
    }
    bool operator==(const Vertex& rhs) const
    {
      return first.get() == rhs.first.get() && second == rhs.second;
    }
  };

  struct opless
  {
    inline bool operator()(const Vertex&lhs, const Vertex& rhs) const
    {
      return lhs.first.get() < rhs.first.get() || (lhs.first.get()
          == rhs.first.get() && lhs.second < rhs.second);
    }
  };

  class label_writer
  {
  public:
    label_writer(const ModuleGraph& graph) :
      graph(graph)
    {
    }
    void operator()(std::ostream& out, const size_t& v) const
    {
      const Vertex& vert = graph.get_vert(v);
      switch (vert.ecto_type)
      {
      case plasm::root:
        out << "[label=\"" << vert.first->name()
            << "\",fillcolor=green, style=\"rounded,filled\"]";
        break;
      case plasm::output:
        out << "[label=\""
            << /* ecto::name_of(typeid(*vert.first)) << "out(" */vert.second
            << "\", shape=invhouse]";
        break;
      case plasm::input:
        out << "[label=\""
            << /*ecto::name_of(typeid(*vert.first)) << "in(" */vert.second
            << "\", shape=house]";

        break;
      case plasm::param:
        break;
      }

    }
  private:
    const ModuleGraph& graph;
  };

  typedef adjacency_list<boost::setS, vecS, boost::bidirectionalS,
      Vertex::Property> graph_t;
  typedef graph_traits<graph_t> GraphTraits;
  typedef graph_traits<graph_t>::vertex_descriptor Vertex_Desc;

  graph_t graph_, root_graph_;

  void add_edge(const module::ptr& from_m, const std::string& out_name,
      const module::ptr& to_m, const std::string& in_name)
  {
    Vertex root_from = make_vert(from_m);
    Vertex root_to = make_vert(to_m);
    Vertex from = make_vert(from_m, out_name, plasm::output);
    Vertex to = make_vert(to_m, in_name, plasm::input);

    boost::add_edge(root_from.uid, root_to.uid, root_graph_);
    boost::add_edge(root_from.uid, from.uid, graph_);
    boost::add_edge(from.uid, to.uid, graph_);
    boost::add_edge(to.uid, root_to.uid, graph_);
  }

  Vertex& get_vert(size_t uid)
  {
    return boost::get(Vertex::Tag(), graph_)[uid];
  }
  const Vertex& get_vert(size_t uid) const
  {
    return boost::get(Vertex::Tag(), graph_)[uid];
  }

  Vertex& get_root_vert(size_t uid)
  {
    return boost::get(Vertex::Tag(), root_graph_)[uid];
  }

  Vertex make_vert(const module::ptr& p, const std::string& s,
      plasm::vertex_t t)
  {
    Vertex v(p, s, t);
    if (getUID(v))
    {
      boost::put(Vertex::Tag(), graph_, v.uid, v);
      //boost::put(Vertex::Tag(),root_graph_,v.uid,v);
    }
    return v;
  }
  Vertex make_vert(const module::ptr& p)
  {
    return make_vert(p, "", plasm::root);
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
      //std::cout << "v : " << v <<   " ptr: " <<     graph.get_vert(v).first<< std::endl;
      //fixme!!!
      if (graph.graph_.out_edge_list(v).empty())
        return;
      GraphTraits::out_edge_iterator out_i, out_end;
      GraphTraits::edge_descriptor e;
      for (boost::tie(out_i, out_end) = boost::out_edges(v, graph.root_graph_); out_i
          != out_end; ++out_i)
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
      //check if the module is dirty, early escape if it is.
      if (!vert.first->dirty())
        return;

      GraphTraits::in_edge_iterator in_i, in_end;
      GraphTraits::edge_descriptor e;
      //verify that the current node is in the graph? WTF (shouldn't in_i == in_end in this case?)
      if (!boost::in_edge_list(graph.graph_, v).empty())
      {
        for (boost::tie(in_i, in_end) = boost::in_edges(v, graph.root_graph_); in_i
            != in_end; ++in_i)
        {
          e = *in_i; //if we don't check and the vertex is not in the graph this causes segfault
          Vertex_Desc targ = boost::source(e, graph.root_graph_);
          (*this)(targ); //recurse.
        }
      }
      //process the module
      vert.first->process();
      //mark dirty for caching.
      vert.first->dirty(false);

    }
    ModuleGraph& graph;
  };

  struct Stacker
  {
    Stacker(ModuleGraph&g) :
      graph(g)
    {
    }
    void operator()(const Vertex_Desc& v)
    {
      if (visited.count(v))
        return; //exit as we already visited.
      GraphTraits::in_edge_iterator in_i, in_end;
      GraphTraits::edge_descriptor e;
      //verify that the current node is in the graph? WTF (shouldn't in_i == in_end in this case?)
      if (!boost::in_edge_list(graph.graph_, v).empty())
      {
        for (boost::tie(in_i, in_end) = boost::in_edges(v, graph.root_graph_); in_i
            != in_end; ++in_i)
        {
          e = *in_i; //if we don't check and the vertex is not in the graph this causes segfault
          Vertex_Desc targ = boost::source(e, graph.root_graph_);
          (*this)(targ); //recurse.
        }
      }
      stack.push_back(graph.get_vert(v).first);
      visited.insert(v);

    }
    ModuleGraph& graph;
    std::vector<module_ptr> stack;
    std::set<Vertex_Desc> visited;
  };
  void mark_dirty(const module::ptr& m)
  {
    Dirtier(*this)(make_vert(m).uid);
  }
  void go(const module::ptr& m)

  {
    Goer(*this)(make_vert(m).uid);
  }

  std::vector<module_ptr> getStack(const module::ptr& output)
  {
    Stacker x(*this);
    x(make_vert(output).uid);
    return x.stack;
  }

  typedef std::set<Vertex, opless> module_set_t;
  module_set_t module_set, module_root_set;

  plasm::vertex_map_t getVertices()
  {
    plasm::vertex_map_t vertices;
    GraphTraits::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(graph_); vi != vi_end; ++vi)
    {
      Vertex& v = get_vert(*vi);
      vertices[*vi] = boost::make_tuple(v.first, v.ecto_type, v.second,
          v.getTendril());
    }
    return vertices;
  }

  plasm::edge_list_t getEdges()
  {
    plasm::edge_list_t list;
    GraphTraits::edge_descriptor e;
    GraphTraits::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei)
    {
      e = *ei;
      list.push_back(
          boost::make_tuple(boost::source(e, graph_), boost::target(e, graph_)));
    }
    return list;
  }

  boost::python::dict getVerticesPy()
  {
    boost::python::dict dict;
    GraphTraits::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(graph_); vi != vi_end; ++vi)
    {
      Vertex& v = get_vert(*vi);
      dict[*vi] = boost::python::make_tuple(v.first, v.ecto_type, v.second,
          v.getTendril());
    }
    return dict;
  }
  boost::python::list getEdgesPy()
  {
    boost::python::list list;
    GraphTraits::edge_descriptor e;
    GraphTraits::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei)
    {
      e = *ei;
      //source = boost::python::make_tuple(v.first,v.second, v.ecto_type);
      list.append(
          boost::python::make_tuple(boost::source(e, graph_),
              boost::target(e, graph_)));
    }
    return list;
  }
  /** Assigns a unique vertex id, from the graph.
   *
   * @param v
   * @return true if new
   */
  inline bool getUID(Vertex& v)
  {
    module_set_t::const_iterator it = module_set.find(v);
    if (it == module_set.end())
    {
      v.uid = boost::add_vertex(graph_);
      module_set.insert(v);
      return true;
    }
    else
    {
      v.uid = it->uid;
      return false;
    }
  }

};

struct plasm::impl
{
  impl() :
    dirty_(true)
  {
  }
  ModuleGraph modules_;
  std::set<module_ptr> inputs_, outputs_;
  typedef std::map<module_ptr, std::vector<module_ptr> > stack_map;
  stack_map p_stacks_;
  void calc_stacks()
  {
    if (dirty_)
    {
      p_stacks_.clear();
      BOOST_FOREACH(module_ptr x, outputs_)
            {
              p_stacks_[x] = modules_.getStack(x);
            }
      dirty_ = false;
    }
  }
  void mark_stacks_dirty()
  {
    BOOST_FOREACH(stack_map::value_type& x, p_stacks_)
          {
            BOOST_FOREACH(module::ptr m, x.second)
                  {
                    m->dirty(true);
                  }
          }
  }
  void proc_stacks()
  {
    BOOST_FOREACH(stack_map::value_type& x, p_stacks_)
    {
      //std::cout << "[";
      BOOST_FOREACH(module::ptr m, x.second)
      {
       // std::cout << modules_.make_vert(m).uid << ",";
        if (!m->dirty())
          continue;
        m->process();
        m->dirty(false);
        //std::cout << "executing: " << m->name() << std::endl;
      }
      //std::cout << "]" <<std::endl;
    }
  }
  bool dirty_;
};

}

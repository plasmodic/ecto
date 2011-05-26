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
#include <boost/graph/topological_sort.hpp>
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
    size_t uid, uid_root;
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
          return first->parameters.at(second);
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
      return lhs.first.get() < rhs.first.get() || (lhs.first.get() == rhs.first.get() && lhs.second < rhs.second);
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
          out << "[label=\"" << vert.first->name() << "\",fillcolor=green, style=\"rounded,filled\"]";
          break;
        case plasm::output:
          out << "[label=\"" << vert.second << "\", shape=invhouse]";
          break;
        case plasm::input:
          out << "[label=\"" << vert.second << "\", shape=house]";
          break;
        case plasm::param:
          break;
        }
    }
  private:
    const ModuleGraph& graph;
  };

  typedef adjacency_list<boost::setS, vecS, boost::bidirectionalS, Vertex::Property> graph_t;
  typedef graph_traits<graph_t> GraphTraits;
  typedef graph_traits<graph_t>::vertex_descriptor Vertex_Desc;
  typedef graph_traits<graph_t>::edge_descriptor Edge_Desc;

  graph_t graph_, root_graph_;

  void add_edge(const module::ptr& from_m, const std::string& out_name, const module::ptr& to_m,
                const std::string& in_name)
  {
    Vertex root_from = make_vert(from_m);
    Vertex root_to = make_vert(to_m);
    Vertex from = make_vert(from_m, out_name, plasm::output);
    Vertex to = make_vert(to_m, in_name, plasm::input);
    if (!boost::in_edge_list(graph_, to.uid).empty())
      {
        throw std::runtime_error(in_name + " is already connected, this is considered an error");
      }
    boost::add_edge(root_from.uid_root, root_to.uid_root, root_graph_);
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

  Vertex make_vert(const module::ptr& p, const std::string& s, plasm::vertex_t t)
  {
    Vertex v(p, s, t);
    if (getUID(v))
      {
        boost::put(Vertex::Tag(), graph_, v.uid, v);
        if (v.ecto_type == plasm::root)
          boost::put(Vertex::Tag(), root_graph_, v.uid_root, v);
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
      graph.get_root_vert(v).first->mark_dirty();
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
    int process(Vertex& vert)
    {
      GraphTraits::in_edge_iterator in_i, in_end;
      GraphTraits::edge_descriptor e, e2;
      //go over all inputs and populate them with the outputs
      for (boost::tie(in_i, in_end) = boost::in_edges(vert.uid, graph.graph_); in_i != in_end; ++in_i)
        {
          e = *in_i; //if we don't check and the vertex is not in the graph this causes segfault
          Vertex input = graph.get_vert(boost::source(e, graph.graph_));
          //there should only be one edge here
          e2 = *(boost::in_edges(input.uid, graph.graph_).first);
          Vertex output = graph.get_vert(boost::source(e2, graph.graph_));
          //sets the input equal to the output
          //std::cout << "copy " << output.second << " to " << input.second << std::endl;
          input.first->inputs.at(input.second).copy_value(output.first->outputs.at(output.second));
        }
      //process the module
      int val = vert.first->process();
      return val;
    }
    int operator()(const Vertex_Desc& v)
    {
      Vertex& vert = graph.get_root_vert(v);
      //check if the module is dirty, early escape if it is.
      if (vert.first->clean())
        return 0;
      GraphTraits::in_edge_iterator in_i, in_end;
      GraphTraits::edge_descriptor e, e2;
      for (boost::tie(in_i, in_end) = boost::in_edges(vert.uid_root, graph.root_graph_); in_i != in_end; ++in_i)
        {
          e = *in_i; //if we don't check and the vertex is not in the graph this causes segfault
          Vertex_Desc targ = boost::source(e, graph.root_graph_);
          int val = (*this)(targ); //recurse.
          if(val)
            return val;
        }
      return process(vert);

    }
    ModuleGraph& graph;
  };

  struct Orderer
  {
    Orderer(ModuleGraph&g) :
      graph(g)
    {

    }
    void operator()()
    {
      typedef std::vector<Vertex_Desc> container;
      container result;
      boost::topological_sort(graph.root_graph_, std::back_inserter(result));

      BOOST_FOREACH(Vertex_Desc x,result)
              {
                //in reverse order
                stack.push_front(x);
              }
    }
    ModuleGraph& graph;
    std::list<Vertex_Desc> stack;
  };

  void mark_dirty(const module::ptr& m)
  {
    Dirtier(*this)(make_vert(m).uid_root);
  }
  int go(const module::ptr& m)
  {
    return Goer(*this)(make_vert(m).uid_root);
  }

  typedef std::set<Vertex, opless> module_set_t;
  module_set_t module_set;

  plasm::vertex_map_t getVertices()
  {
    plasm::vertex_map_t vertices;
    GraphTraits::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(graph_); vi != vi_end; ++vi)
      {
        Vertex& v = get_vert(*vi);
        vertices[*vi] = boost::make_tuple(v.first, v.ecto_type, v.second, v.getTendril());
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
        list.push_back(boost::make_tuple(boost::source(e, graph_), boost::target(e, graph_)));
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
        dict[*vi] = boost::python::make_tuple(v.first, v.ecto_type, v.second, v.getTendril());
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
        list.append(boost::python::make_tuple(boost::source(e, graph_), boost::target(e, graph_)));
      }
    return list;
  }

  inline bool getUID(Vertex& v)
  {
    module_set_t::const_iterator it = module_set.find(v);
    if (it == module_set.end())
      {
        v.uid = boost::add_vertex(graph_);
        if (v.ecto_type == plasm::root)
          v.uid_root = boost::add_vertex(root_graph_);
        else
          v.uid_root = v.uid;
        module_set.insert(v);
        return true;
      }
    else
      {
        v.uid = it->uid;
        v.uid_root = it->uid_root;
        return false;
      }
  }

};

struct plasm::impl
{
  impl() :
    dirty_(true), finished_(false)
  {
  }

  void calc_stacks()
  {
    if (dirty_)
      {
        ModuleGraph::Orderer o(modules_);
        o();
        stack_ = o.stack;
        dirty_ = false;
      }
  }
  void mark_stacks_dirty()
  {
    BOOST_FOREACH(ModuleGraph::Vertex_Desc desc, stack_)
      {
        module::ptr m = modules_.get_root_vert(desc).first;
        m->mark_dirty();
      }
  }

  int proc_stacks()
  {
    ModuleGraph::Goer goer(modules_);

    BOOST_FOREACH(ModuleGraph::Vertex_Desc desc, stack_)
      {
        int val =goer.process(modules_.get_root_vert(desc));
        if (val)
          return 1;
      }
    return 0;
  }

  void signal_finished()
  {
    finished_ = true;
  }

  bool dirty_, finished_;
  ModuleGraph modules_;
  std::list<ModuleGraph::Vertex_Desc> stack_;
};

}

#include <stdlib.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/atomic.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/shared_array.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>
//this quiets a deprecated warning
#define BOOST_NO_HASH
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <deque>

boost::atomic<unsigned> nsimultaneous, noutstanding;



namespace asio = boost::asio;
asio::io_service io_service;
boost::thread_group tgroup;

struct module
{
  typedef std::map<std::string, boost::any> tendrils_t;
  virtual void process(tendrils_t& in, tendrils_t& out) = 0;
  typedef boost::shared_ptr<module> ptr;
  std::string name;
  module(const std::string& name_) : name(name_) { }

  virtual ~module();
};

module::~module() { }

struct generator : module
{
  float value;
  generator(const std::string& name) : module(name)
  { 
    value = 0.0f; 
  }

  void process(tendrils_t& in, tendrils_t& out)
  {
    out["value"] = value;
    value += 1.0f;
  }
};

struct incrementer : module
{
  incrementer(const std::string& name) : module(name) { }

  void process(tendrils_t& in, tendrils_t& out)
  {
    float val = boost::any_cast<float>(in["value"]);
    out["value"] = val + 1.0f;
  }
};

struct splitter : module
{
  splitter(const std::string& name) : module(name) { }

  void process(tendrils_t& in, tendrils_t& out)
  {
    float val = boost::any_cast<float>(in["value"]);
    out["left"] = val;
    out["right"] = val;
  }
};

struct adder : module
{
  adder(const std::string& name) : module(name) { }

  void process(tendrils_t& in, tendrils_t& out)
  {
    float left = boost::any_cast<float>(in["left"]);
    float right = boost::any_cast<float>(in["right"]);
    out["value"] = left + right;
  }
};


using boost::adjacency_list;
using boost::vecS;
using boost::bidirectionalS;
using boost::graph_traits;

struct edge 
{
  edge(const std::string& fp, const std::string& tp)
    : from_port(fp), to_port(tp)
  { }

  std::string from_port, to_port;
  std::deque<boost::any> deque;
  typedef boost::shared_ptr<edge> ptr;
  typedef boost::shared_ptr<const edge> const_ptr;
};


// if the first argument is a sequence type (vecS, etc) then parallel edges are allowed
typedef adjacency_list<vecS, // OutEdgeList... 
                       vecS, // VertexList
                       bidirectionalS, // Directed
                       module::ptr, // vertex property
                       edge::ptr> // edge property 
graph_t;
                       
struct vertex_writer
{
  graph_t* g;
  vertex_writer(graph_t* g_) : g(g_) { }

  void operator()(std::ostream& out, graph_t::vertex_descriptor vd)
  {
    out << "[label=\"" << (*g)[vd]->name << "\"]";
  }
};

struct edge_writer
{
  graph_t* g;
  edge_writer(graph_t* g_) : g(g_) { }

  void operator()(std::ostream& out, graph_t::edge_descriptor ed)
  {
    out << "[headlabel=\"" << (*g)[ed]->to_port << "\" taillabel=\"" << (*g)[ed]->from_port << "\"]";
  }
};

struct graph_writer
{
  void operator()(std::ostream& out) const {
    out << "graph [rankdir=LR, ranksep=1]" << std::endl;
    out << "edge [labelfontsize=8]" << std::endl;
  }
};

edge::ptr make_edge(const std::string& fromport, const std::string& toport)
{
  edge::ptr eptr(new edge(fromport, toport));
  return eptr;
}

int main()
{
  using boost::bind;
  namespace asio = boost::asio;

  std::cout << "start..." << std::endl;
  
  graph_t g;

  module::ptr 
    gen(new generator("generate")), 
    split(new splitter("split")),
    inc(new incrementer("increment")),
    add(new adder("add"))
    ;
  
  graph_t::vertex_descriptor 
    gen_d = add_vertex(gen, g),
    split_d = add_vertex(split, g),
    inc_d = add_vertex(inc, g),
    add_d = add_vertex(add, g)
    ;

  bool added;
  graph_t::edge_descriptor ed;
  tie(ed, added) = add_edge(gen_d, split_d, make_edge("value", "value"), g);
  assert(added);

  tie(ed, added) = add_edge(split_d, inc_d, make_edge("left", "value"), g);
  assert(added);

  tie(ed, added) = add_edge(split_d, add_d, make_edge("right", "right"), g);
  assert(added);

  tie(ed, added) = add_edge(inc_d, add_d, make_edge("value", "left"), g);
  assert(added);

  std::ofstream gviz_str("graph.dot");
  boost::write_graphviz(gviz_str, g, vertex_writer(&g), edge_writer(&g), graph_writer());

  /*
  module::ptr graph = make_graph(io_service, 30);

  graph->run(io_service);

  {
    for (unsigned j=0; j<1; ++j)
      {
        tgroup.create_thread(bind(&asio::io_service::run, &io_service));
      }
  }

  tgroup.join_all();
  */
  std::cout << "exit." << std::endl;
}

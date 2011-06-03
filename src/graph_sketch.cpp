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

typedef std::map<std::string, boost::any> tendrils_t;

struct module
{
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

struct printer : module
{
  printer(const std::string& name) : module(name) { }

  void process(tendrils_t& in, tendrils_t& out)
  {
    float value = boost::any_cast<float>(in["value"]);
    std::cout << "VALUE=" << value << "\n";
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

void invoke_process(graph_t& g, graph_t::vertex_descriptor vd)
{
  std::cout << __PRETTY_FUNCTION__ << "  " << g[vd]->name << "\n";
  module::ptr m = g[vd];
  tendrils_t tmp_inputs, tmp_outputs;
  
  graph_t::in_edge_iterator inbegin, inend;
  tie(inbegin, inend) = boost::in_edges(vd, g);
  while(inbegin != inend) {
    edge::ptr e = g[*inbegin];
    std::cout << "IN: " << e->from_port << " => " << e->to_port << "\n";

    tmp_inputs[e->to_port] = e->deque.front();
    e->deque.pop_front();
    ++inbegin;
  }

  m->process(tmp_inputs, tmp_outputs);

  // build temporary outedge map
  std::map<std::string, edge::ptr> outedges;
  graph_t::out_edge_iterator outbegin, outend;
  tie(outbegin, outend) = boost::out_edges(vd, g);
  while(outbegin != outend) 
    {
      edge::ptr e = g[*outbegin];
      outedges[e->from_port] = e;
      std::cout << e->from_port << " => " << e << "\n";
      ++outbegin;
    }

  for (tendrils_t::iterator it = tmp_outputs.begin(), end = tmp_outputs.end();
       it != end; ++it)
    {
      std::cout << "push to " << it->first << "\n";
      outedges[it->first]->deque.push_back(it->second);
    }
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
    add(new adder("add")),
    print(new printer("print"))
    ;
  
  graph_t::vertex_descriptor 
    add_d = add_vertex(add, g),
    inc_d = add_vertex(inc, g),
    split_d = add_vertex(split, g),
    gen_d = add_vertex(gen, g),
    print_d = add_vertex(print, g)
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

  tie(ed, added) = add_edge(add_d, print_d, make_edge("value", "value"), g);
  assert(added);

  std::ofstream gviz_str("graph.dot");
  boost::write_graphviz(gviz_str, g, vertex_writer(&g), edge_writer(&g), graph_writer());

  std::vector<graph_t::vertex_descriptor> vv;

  boost::topological_sort(g, std::back_inserter(vv));
  std::reverse(vv.begin(), vv.end());

  for (unsigned j=0; j<5; ++j)
    {
      std::cout << "**************************************************\n";
      for (unsigned k=0; k<vv.size(); ++k)
        {
          invoke_process(g, vv[k]);
        }
    }
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

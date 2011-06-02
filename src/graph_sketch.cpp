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

struct data {
  float value;
  typedef boost::shared_ptr<data> ptr;
  typedef boost::shared_ptr<const data> const_ptr;
};

typedef std::deque<data::const_ptr> datadeque;

struct module : boost::enable_shared_from_this<module> , boost::noncopyable
{
  typedef boost::shared_ptr<module> ptr;
  typedef std::map<std::string, datadeque> inputs_t;
  inputs_t inputs;

  typedef std::map<std::string, std::pair<ptr, std::string> > outputs_t;
  outputs_t outputs;

  boost::mutex mtx;

  std::string name;

  module(const std::string& name_) : name(name_) { }

  bool inputs_ready() 
  {
    boost::unique_lock<boost::mutex> lock(mtx);
    if (inputs.size() == 0)
      {
        std::cout << this << " READY CAUSE NO INPUTS\n";
        return true;
      }
    for (inputs_t::iterator it = inputs.begin(), end = inputs.end();
         it != end;
         ++it)
      {
        if (it->second.size() == 0)
          return false;
      }
    return true;
  }

  void push(const std::string& port, data::const_ptr newdata)
  {
    boost::unique_lock<boost::mutex> lock(mtx);
    inputs[port].push_back(newdata);
  }

  void doit(boost::asio::io_service& serv) 
  {
    data::ptr newdata(new data);
    newdata->value = 1;
    {
      boost::unique_lock<boost::mutex> lock(mtx);
      for (inputs_t::iterator it = inputs.begin(), end = inputs.end();
           it != end;
           ++it)
        {
          newdata->value += it->second.front()->value;
          it->second.pop_front();
        }
    }
    std::cout << this << " new value = " << newdata->value << "\n";
    std::cout << this << " pushing to " << outputs.size() << " outputs" << std::endl;
    // spend some time working
    unsigned ms_work = rand() % 3000;
    boost::asio::io_service inner_serv;
    boost::asio::deadline_timer dt(inner_serv);
    dt.expires_from_now(boost::posix_time::milliseconds(ms_work));
    dt.wait();

    for (outputs_t::iterator it = outputs.begin(), end = outputs.end();
         it != end;
         ++it)
      {
        module::ptr downstream_module = it->second.first;
        const std::string& downstream_port = it->second.second;

        std::cout << this << " => " << downstream_port << "\n";
        downstream_module->push(downstream_port, newdata);
      }

    run(serv);
  }

  static void 
  async_waitforinput(boost::asio::io_service& serv,
                     module::ptr m)
  {
    std::cout << m.get() << " waiting..." << std::endl;
    boost::asio::io_service inner_serv;
    boost::asio::deadline_timer dt(inner_serv);
    boost::asio::io_service::work work(serv);

    assert(m.get());
      
    for (;;) {
      if (m->inputs_ready())
        {
          serv.post(boost::bind(&module::doit, m.get(), boost::ref(serv)));
          std::cout << m.get() << " inputs ready! " << m->inputs.size() << std::endl;
          return;
        }
      dt.expires_from_now(boost::posix_time::milliseconds(3));
      dt.wait();
    }
  }
                                 
  boost::scoped_ptr<boost::thread> input_watcher;

  void run(boost::asio::io_service& serv)
  {
    if (input_watcher)
      input_watcher->join();
    input_watcher.reset(new boost::thread(boost::bind(&async_waitforinput,
                                                      boost::ref(serv), 
                                                      shared_from_this())));
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

int main()
{
  using boost::bind;
  namespace asio = boost::asio;

  std::cout << "start..." << std::endl;
  
  graph_t g;

  module::ptr m1(new module("m1")), m2(new module("m2"));
  edge::ptr e1(new edge("out1", "in1")), e2(new edge("out2", "in2"));

  graph_t::vertex_descriptor vd1 = add_vertex(m1, g),
    vd2 = add_vertex(m2, g);

  bool added;
  graph_t::edge_descriptor ed1, ed2;
  tie(ed1, added) = add_edge(vd1, vd2, e1, g);
  assert(added);
  tie(ed2, added) = add_edge(vd1, vd2, e2, g);
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

#include <ecto/plasm.hpp>
#include "plasm/impl.hpp"
#include <ecto/tendril.hpp>
#include <ecto/cell.hpp>
#include <ecto/graph_types.hpp>
#include <ecto/schedulers/singlethreaded.hpp>

#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/regex.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

namespace ecto
{

  using namespace graph;
  #define STRINGY_DINGY(A) #A
  //see http://www.graphviz.org/content/node-shapes for reference.
  const char* table_str = STRINGY_DINGY(
      <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4">
      %s
      <TR>
      %s
      %s
      </TR>
      %s
      %s
      </TABLE>
  );

  const char* input_str = STRINGY_DINGY(
      <TD PORT="i_%s" BGCOLOR="springgreen">%s<BR/><FONT POINT-SIZE="8">%s</FONT></TD>
  );

  const char* cell_str = 
    STRINGY_DINGY(<TD ROWSPAN="%d" COLSPAN="%d" BGCOLOR="khaki">%s<BR/><FONT POINT-SIZE="8">%s</FONT></TD>
  );

  const char* param_str_1st = STRINGY_DINGY(
      <TD PORT="p_%s" BGCOLOR="lightblue">%s<BR/><FONT POINT-SIZE="8">%s</FONT></TD>
  );

  const char* param_str_N = STRINGY_DINGY(
      <TR>
      <TD PORT="p_%s" BGCOLOR="lightblue">%s<BR/><FONT POINT-SIZE="8">%s</FONT></TD>
      </TR>
  );

  const char* output_str = STRINGY_DINGY(
      <TD PORT="o_%s" BGCOLOR="indianred1">%s<BR/><FONT POINT-SIZE="8">%s</FONT></TD>
  );

  struct vertex_writer
  {
    graph_t* g;

    vertex_writer(graph_t* g_)
        :
          g(g_)
    {
    }

    std::string htmlescape(const std::string& in)
    {
      const boost::regex esc_lt("[<]");
      const std::string rep_lt("&lt;");
      const boost::regex esc_gt("[>]");
      const std::string rep_gt("&gt;");

      std::string htmlescaped_name = in;
      htmlescaped_name = boost::regex_replace(htmlescaped_name, esc_lt, rep_lt, boost::match_default);
      htmlescaped_name = boost::regex_replace(htmlescaped_name, esc_gt, rep_gt, boost::match_default);
      return htmlescaped_name;
    }

    void
    operator()(std::ostream& out, graph_t::vertex_descriptor vd)
    {


      cell::ptr c = (*g)[vd];
      int n_inputs = c->inputs.size();
      int n_outputs = c->outputs.size();
      int n_params = c->parameters.size();
      std::string htmlescaped_name = htmlescape(c->name());

      std::string inputs;
      BOOST_FOREACH(const tendrils::value_type& x, c->inputs)
          {
            std::string key = x.first;
            if (inputs.empty())
              inputs = "<TR>\n";
            inputs += boost::str(boost::format(input_str) % key % key % htmlescape(x.second->type_name())) + "\n";
          }
      if (!inputs.empty())
        inputs += "</TR>";

      std::string outputs;
      BOOST_FOREACH(const tendrils::value_type& x, c->outputs)
          {
            std::string key = x.first;
            if (outputs.empty())
              outputs = "<TR>\n";
            outputs += boost::str(boost::format(output_str) % key % key % htmlescape(x.second->type_name())) + "\n";
          }
      if (!outputs.empty())
        outputs += "</TR>";

      std::string cellrow = boost::str(
          boost::format(cell_str) 
          % std::max(1,n_params) 
          % int(std::max(1,std::max(n_inputs, n_outputs))) 
          % htmlescaped_name
          % htmlescape(c->type()));
      std::string p1, pN;
      BOOST_FOREACH(const tendrils::value_type& x, c->parameters)
          {
            std::string key = x.first;
            if (p1.empty())
              p1 = boost::str(boost::format(param_str_1st) % key % key % htmlescape(x.second->type_name())) + "\n";
            else
              pN += boost::str(boost::format(param_str_N) % key % key % htmlescape(x.second->type_name())) + "\n";
          }

      std::string table = boost::str(boost::format(table_str) % inputs % cellrow % p1 % pN % outputs);
      out << "[label=<" << table << ">]";
    }
  };

  struct edge_writer
  {
    graph_t* g;

    edge_writer(graph_t* g_)
        :
          g(g_)
    {
    }

    void
    operator()(std::ostream& out, graph_t::edge_descriptor ed)
    {
      out << "[headport=\"i_" << (*g)[ed]->to_port << "\" tailport=\"o_" << (*g)[ed]->from_port << "\"]";
    }
  };

  struct graph_writer
  {
    void
    operator()(std::ostream& out) const
    {
      out << "graph [rankdir=TB, ranksep=1]" << std::endl;
      out << "edge [labelfontsize=8]" << std::endl;
      out << "node [shape=plaintext]" << std::endl;
    }
  };

  plasm::plasm() : impl_(new impl) { }


  plasm::~plasm() { }

  void
  plasm::insert(cell::ptr mod)
  {
    impl_->insert_module(mod);
  }

  void
  plasm::connect(cell::ptr from, const std::string& output, cell::ptr to, const std::string& input)
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
  plasm::disconnect(cell_ptr from, const std::string& output, cell_ptr to, const std::string& input)
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
    if (!impl_->scheduler)
      impl_->scheduler.reset(new ecto::schedulers::singlethreaded(shared_from_this()));
    impl_->scheduler->execute(niter);
    while (impl_->scheduler->running())
      boost::this_thread::sleep(boost::posix_time::microseconds(10));

    return 0;
  }

  std::size_t 
  plasm::size() const
  {
    return num_vertices(impl_->graph);
  }

  std::vector<cell::ptr>
  plasm::cells() const
  {
    std::vector<cell::ptr> c;
    std::transform(impl_->mv_map.begin(), impl_->mv_map.end(), std::back_inserter(c), impl::CVMtoCell());
    return c;
  }
  void plasm::configure_all()
  {
    BOOST_FOREACH(impl::ModuleVertexMap::value_type& x, impl_->mv_map)
    {
      x.first->configure();
    }
  }

  void
  plasm::check() const
  {
    graph_t& g(impl_->graph);
    graph_t::vertex_iterator begin, end;
    tie(begin, end) = boost::vertices(g);
    while (begin != end)
    {
      cell::ptr m = g[*begin];
      std::set<std::string> in_connected, out_connected;

      //verify all required inputs are connected
      graph_t::in_edge_iterator b_in, e_in;
      tie(b_in, e_in) = boost::in_edges(*begin, g);
      while (b_in != e_in)
      {
        edge::ptr in_edge = g[*b_in];
        cell::ptr from_module = g[source(*b_in, g)];
        in_connected.insert(in_edge->to_port);
        ++b_in;
      }

      for (tendrils::const_iterator b_tend = m->inputs.begin(), e_tend = m->inputs.end(); b_tend != e_tend; ++b_tend)
      {

        if (b_tend->second->required() && in_connected.count(b_tend->first) == 0)
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
        if (b_tend->second->required() && out_connected.count(b_tend->first) == 0)
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

#pragma once

#include <ecto/plasm.hpp>
#include <ecto/registry.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/nvp.hpp>
#include <ecto/serialization/cell.hpp>

namespace ecto
{
  namespace serialization
  {
    typedef ecto::graph::graph_t::vertex_descriptor vd_t;
    typedef boost::tuple<vd_t, std::string, vd_t, std::string> connection_t;
    typedef std::map<vd_t, cell::ptr> cell_map_t;
  }

  template<class Archive>
  void
  plasm::save(Archive & ar, const unsigned int version) const
  {
    const ecto::graph::graph_t& g = graph();
    ecto::graph::graph_t::edge_iterator begin, end;
    serialization::vd_t source, sink;
    serialization::cell_map_t cell_map;
    std::vector<serialization::connection_t> connections;
    for (boost::tie(begin, end) = boost::edges(g); begin != end; ++begin)
    {
      source = boost::source(*begin, g);
      sink = boost::target(*begin, g);
      cell::ptr to = g[sink], from = g[source];
      cell_map[sink] = to;
      cell_map[source] = from;
      std::string to_port = g[*begin]->to_port;
      std::string from_port = g[*begin]->from_port;
      connections.push_back(boost::make_tuple(source, from_port, sink, to_port));
    }
    ar << cell_map;
    ar << connections;
  }

  template<class Archive>
  void
  plasm::load(Archive & ar, const unsigned int version)
  {
    serialization::cell_map_t cell_map;
    std::vector<serialization::connection_t> connections;
    ar >> cell_map;
    ar >> connections;
    for (size_t i = 0; i < connections.size(); i++)
    {
      cell::ptr from, to;
      from = cell_map[connections[i].get<0>()];
      to = cell_map[connections[i].get<2>()];
      std::string from_port, to_port;
      from_port = connections[i].get<1>();
      to_port = connections[i].get<3>();
      connect(from, from_port, to, to_port);
    }
  }
}

namespace boost
{
  namespace serialization
  {
    template<typename Archive>
    void
    serialize(Archive& ar, 
              ecto::serialization::connection_t& c,
              const unsigned int version )
    {
      ar & c.get<0>();
      ar & c.get<1>();
      ar & c.get<2>();
      ar & c.get<3>();
    }
  }
}

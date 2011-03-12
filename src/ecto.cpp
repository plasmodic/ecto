#include <ecto/util.hpp>
#include <ecto/ecto.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/lexical_cast.hpp>

#include <map>
#include <set>
#include <sstream>

namespace ecto
{
  connection::connection() :
    dirty_(true)
  {
  }
  connection::connection(impl_base::ptr impl) :
    impl_(impl), dirty_(true)
  {
  }
  std::string connection::type_name() const
  {
    return impl_->type_name();
  }
  std::string connection::value() const
  {
    return impl_->value();
  }
  std::string connection::name() const
  {
    return impl_->name;
  }
  std::string connection::doc() const
  {
    return impl_->doc;
  }

  void connection::connect(connection& rhs)
  {
    impl_ = rhs.impl_;
  }

  connection::impl_base::~impl_base()
  {
  }
  module::module() :
    dirty_(true)
  {
  }
  module::~module()
  {
  }
  void module::Process()
  {
  }

  void module::connect(const std::string& out_name, ptr to, const std::string& in_name)
  {
    to->inputs[in_name].connect(outputs[out_name]);
  }

  void module::dirty(bool mark)
  {
    for (connections_t::iterator it = outputs.begin(), end = outputs.end(); it != end; ++it)
    {
      it->second.dirty(mark);
    }
    dirty_ = mark;
  }
  bool module::dirty() const
  {
    for (connections_t::const_iterator it = inputs.begin(), end = inputs.end(); it != end; ++it)
    {
      if (it->second.dirty())
        return true;
    }

    return dirty_;
  }
  void plasm::connect(module::ptr from, const std::string& out_name, module::ptr to, const std::string& in_name)
  {
    edge& f_e = edge_map[from];
    edge& t_e = edge_map[to];
    from->connect(out_name, to, in_name);
    f_e.downstream.insert(to);
    t_e.upstream.insert(from);
  }
  void plasm::markDirty(module::ptr m)
  {
    m->dirty(true);
    edge& edges = edge_map[m];
    for (edge::iterator it = edges.downstream.begin(), end = edges.downstream.end(); it != end; ++it)
    {
      (*it)->dirty(true);
    }
  }
  void plasm::go(module::ptr m)
  {
    if (!m->dirty())
      return;
    edge& edges = edge_map[m];
    for (edge::iterator it = edges.upstream.begin(), end = edges.upstream.end(); it != end; ++it)
    {
      go(*it);
    }
    m->Process();
    m->dirty(false);
  }

  //this grabs the name of the underlying type and appends the address as a unique name
#define PRINT_NAME(_x_i_) name_of(typeid(*(_x_i_))) << "_" << int(0xff & reinterpret_cast<size_t>(&(*(_x_i_))))
  std::string plasm::viz() const
  {
    std::stringstream ss;

    for (map_t::const_iterator it = edge_map.begin(), end = edge_map.end(); it != end; ++it)
    {
      if (it->second.downstream.empty())
        continue;
      ss << PRINT_NAME((it->first)) << " -> {";
      for (edge::const_iterator dit = it->second.downstream.begin(), end = it->second.downstream.end(); dit != end; ++dit)
      {
        ss << PRINT_NAME(*dit) << " ";
      }
      ss << "};\n";
    }
    return ss.str();
  }

}


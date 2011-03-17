#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/module.hpp>
namespace ecto
{
  void plasm::connect(module::ptr from, const std::string& out_name, module::ptr to, const std::string& in_name)
  {
    edge& from_e = edge_map[from];
    edge& to_e = edge_map[to];
    from->connect(out_name, to, in_name);
    from_e.downstream[out_name].insert(to);
    to_e.upstream[in_name] = from;
  }
  void plasm::markDirty(module::ptr m)
  {
    m->dirty(true);
    edge& edges = edge_map[m];
    for (edge::ds::iterator it = edges.downstream.begin(), end = edges.downstream.end(); it != end; ++it)
    {
      for (edge::ds::module_set::iterator mit = it->second.begin(), mend = it->second.end(); mit != mend; ++mit)
      {
        markDirty(*mit);
      }
    }
  }
  void plasm::go(module::ptr m)
  {
    if (!m->dirty())
      return;
    edge& edges = edge_map[m];
    for (edge::us::iterator it = edges.upstream.begin(), end = edges.upstream.end(); it != end; ++it)
    {
      go(it->second);
    }
    m->Process();
    m->dirty(false);
  }

  //this grabs the name of the underlying type and appends the address as a unique name
#define PRINT_NAME(_x_i_) name_of(typeid(*(_x_i_))) << "_" << int(0xffff & reinterpret_cast<size_t>(&(*(_x_i_))))
  std::string plasm::viz() const
  {
    std::stringstream ss;

    for (map_t::const_iterator it = edge_map.begin(), end = edge_map.end(); it != end; ++it)
    {
      if (it->second.downstream.empty())
        continue;
      ss << PRINT_NAME((it->first)) << " -> {";
      for (edge::ds::const_iterator dit = it->second.downstream.begin(), dend = it->second.downstream.end(); dit
          != dend; ++dit)
      {
        for (edge::ds::module_set::iterator mit = dit->second.begin(), mend = dit->second.end(); mit != mend; ++mit)
        {
          ss << PRINT_NAME(*mit) << " ";
        }

      }
      ss << "};\n";
    }
    return ss.str();
  }
}

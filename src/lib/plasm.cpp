#include "plasm_impl.hpp"
namespace ecto
{
plasm::plasm() :
  impl_(new impl)
{
}

void plasm::connect(module::ptr from, const std::string& out_name, module::ptr to, const std::string& in_name)
{
  if (from->outputs.count(out_name) == 0)
  {
    throw std::logic_error("The specified output does not exist: " + out_name);
  }
  if (to->inputs.count(in_name) == 0)
  {
    throw std::logic_error("The specified input does not exist: " + in_name);
  }

  //this throws on bad connection.
  from->connect(out_name, to, in_name);

  //only add to graph if it was actually connected.
  impl_->modules_.add_edge(from, out_name, to, in_name);
  impl_->dirty_ = true;
}

void plasm::mark_dirty(module::ptr m)
{
  // Access the property accessor type for this graph
  impl_->modules_.mark_dirty(m);

}
void plasm::go(module::ptr m)
{
  impl_->modules_.go(m);
}

void plasm::viz(std::ostream& out) const
{
  boost::write_graphviz(out, impl_->modules_.graph_, ModuleGraph::label_writer(impl_->modules_));
}

std::string plasm::viz() const
{
  std::stringstream ss;
  viz(ss);
  return ss.str();
}

plasm::vertex_map_t plasm::getVertices()
{
  return impl_->modules_.getVertices();
}
plasm::edge_list_t plasm::getEdges()
{
  return impl_->modules_.getEdges();
}

void plasm::execute()
{
  impl_->calc_stacks();
  impl_->mark_stacks_dirty();
  impl_->proc_stacks();
}

void plasm::disconnect(module_ptr from, const std::string& output, module_ptr to, const std::string& input)
{
  std::pair<ModuleGraph::Edge_Desc, bool> e = boost::edge(impl_->modules_.make_vert(from, output, plasm::output).uid,
      impl_->modules_.make_vert(to, input, plasm::input).uid, impl_->modules_.graph_);
  if (e.second)
    boost::remove_edge(e.first, impl_->modules_.graph_);
  else
    throw std::runtime_error(from->name() + ":" + output + " not connected to " + to->name() + ":" + input);
  from->outputs[output].disconnect();
  to->inputs[input].disconnect();
}
}

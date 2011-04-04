#include "plasm_impl.hpp"
namespace ecto
{

  plasm::plasm() :
    impl_(new impl)
  {
  }
  void plasm::connect(module::ptr from, const std::string& out_name, module::ptr to, const std::string& in_name)
  {
    impl_->modules_.add_edge(from, out_name, to, in_name);
    from->connect(out_name, to, in_name);
  }

  void plasm::markDirty(const module::ptr& m)
  {
    // Access the property accessor type for this graph
    impl_->modules_.mark_dirty(m);

  }
  void plasm::go(const module::ptr& m)
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

  struct PlasmModule : ecto::module
  {
    PlasmModule(boost::shared_ptr<plasm> plasm, const std::list<module::ptr>& mi, const std::list<module::ptr>& mo) :
      plasm_(plasm), module_inputs_(mi), module_outputs_(mo)
    {
    }

    virtual void connect(const std::string& output, ptr to, const std::string& input)
    {

    }
    virtual void dirty(bool hmm)
    {
      std::cout << "dirty ("<< hmm<<")"<< std::endl;
      ecto::module::dirty(hmm);
    }

    virtual void Process()
    {
      BOOST_FOREACH(const module::ptr& m , module_inputs_)
     {
       plasm_->markDirty(m);
     }
      BOOST_FOREACH(module::ptr& m , module_inputs_)
      {
        BOOST_FOREACH(tendrils::value_type& t , m->inputs)
        {
          t.second.connect(inputs[t.first]);
          std::cout << "connecting: " << t.first << std::endl;
        }
      }
      BOOST_FOREACH(const module::ptr& m , module_outputs_)
      {
        plasm_->go(m);
      }
    }

    virtual void Config()
    {
      BOOST_FOREACH(module::ptr& m , module_inputs_)
      {
        BOOST_FOREACH(tendrils::value_type& t , m->inputs)
        {
          inputs[t.first] = t.second;
        }
      }

      BOOST_FOREACH(const module::ptr& m , module_outputs_)
      {
        BOOST_FOREACH(const tendrils::value_type& t , m->outputs)
        {
          outputs[t.first] = t.second;
        }
      }
    }
    boost::shared_ptr<plasm> plasm_;
    std::list<module::ptr> module_inputs_, module_outputs_;
  };

  boost::shared_ptr<module> plasm::toModule(boost::shared_ptr<plasm> plasm, const std::list<module::ptr>& mi,
                                            const std::list<module::ptr>& mo)
  {
    boost::shared_ptr<module> plasm_ (new PlasmModule(plasm, mi, mo));
    plasm_->Config();
    return plasm_;
  }
}

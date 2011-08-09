#include <ecto/ecto.hpp>
#include <ecto/cell.hpp>

#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <boost/python/raw_function.hpp>
#include <boost/python/iterator.hpp>
#include <boost/python/slice.hpp>
#include <boost/python/stl_iterator.hpp>

#include <ecto/python/std_map_indexing_suite.hpp>
#include <ecto/python/raw_constructor.hpp>
#include <ecto/python/repr.hpp>
#include <string>
namespace bp = boost::python;

namespace ecto
{
  namespace py
  {

    struct cellwrap: cell, bp::wrapper<cell>
    {

      void dispatch_declare_params(tendrils& params)
      {
        if (bp::override init = this->get_override("declare_params"))
          init(boost::ref(params));
      }

      void dispatch_declare_io(const tendrils&params, tendrils& inputs, tendrils& outputs)
      {
        if (bp::override declare_io = this->get_override("declare_io"))
          declare_io(boost::ref(params), boost::ref(inputs), boost::ref(outputs));
      }

      void dispatch_configure(tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        if (bp::override config = this->get_override("configure"))
          config(boost::ref(params));
      }

      struct YouveBeenServed
      {
        void operator()(tendrils::value_type& t)
        {
          t.second->notify();
        }
      };

      ReturnCode dispatch_process(tendrils& inputs, tendrils& outputs)
      {
        std::for_each(inputs.begin(),inputs.end(), YouveBeenServed());
        if (bp::override proc = this->get_override("process"))
          {
            proc(boost::ref(inputs), boost::ref(outputs));
          }
        std::for_each(outputs.begin(),outputs.end(),YouveBeenServed());
        return OK;
      }

      void init() { }

      std::string dispatch_name() const
      {
        bp::reference_existing_object::apply<cellwrap*>::type converter;
        PyObject* obj = converter(this);
        bp::object real_obj = bp::object(bp::handle<>(obj));
        bp::object n = real_obj.attr("__class__").attr("__name__");
        std::string nm = bp::extract<std::string>(n);
        return nm;
      }

      static std::string doc(cellwrap* mod)
      {
        bp::reference_existing_object::apply<cellwrap*>::type converter;
        PyObject* obj = converter(mod);
        bp::object real_obj = bp::object(bp::handle<>(obj));
        bp::object n = real_obj.attr("__class__").attr("__doc__");
        bp::extract<std::string> get_str(n);
        if (get_str.check())
          return get_str();
        return "No Doc str.";
      }
    };

    const tendrils& inputs(cell& mod)
    {
      return mod.inputs;
    }
    tendrils& outputs(cell& mod)
    {
      return mod.outputs;
    }
    tendrils& params(cell& mod)
    {
      return mod.parameters;
    }

    struct TendrilSpecification
    {
      cell::ptr mod_input, mod_output;
      std::string key;

      TendrilSpecification() { }

      bool check(cell::ptr mod, const std::string& key)
      {
        if (key.empty())
          return true;
        if (mod->inputs.find(key) == mod->inputs.end() && mod->outputs.find(key) == mod->outputs.end()
            && mod->parameters.find(key) == mod->parameters.end())
          {
            return false;
          }
        return true;
      }

      TendrilSpecification(cell::ptr mod_in, cell::ptr mod_out, const std::string& key) 
        : mod_input(mod_in), mod_output(mod_out), key(key)
      {
        if (!check(mod_in, key))
          throw std::runtime_error(
          "The module " + mod_in->name() + " does not contain any input or parameter by the given name: "
              + key);
        if (!check(mod_out, key))
          throw std::runtime_error(
          "The module " + mod_out->name() + " does not contain any output or parameter by the given name: "
              + key);
      }

      TendrilSpecification(cell::ptr mod, const std::string& key) 
        : mod_input(mod), mod_output(mod), key(key)
      {
        if (!check(mod, key))
          throw std::runtime_error(
              "The module " + mod->name() + " does not contain any inputs or outputs or parameters by the given name: "
              + key);
      }

      tendril::ptr toTendril(int t)
      {
        switch (t)
          {
          case 0:
            return mod_output->outputs[key];
          case 1:
            return mod_input->inputs[key];
          case 2:
            return mod_input->parameters[key];
          default:
            return tendril::ptr();
          }
      }
      bp::str __str__()
      {
        bp::str str = bp::str(mod_input->name());
        str += ", " + bp::str(key);
        return str;
      }
    };

    struct TendrilSpecifications
    {
      typedef std::vector<TendrilSpecification> Vector;

      TendrilSpecifications() { }

      TendrilSpecifications(Vector vts) : vts(vts) { }

      TendrilSpecifications(bp::list l)
      {
        bp::stl_input_iterator<const TendrilSpecification&> begin(l), end;
        std::copy(begin, end, std::back_inserter(vts));
      }

      TendrilSpecification toSpec()
      {
        if (vts.size() != 1)
          {
            throw std::runtime_error("This specification must be of length one. e.g. module['only one key']");
          }
        return vts.front();
      }

      static tendrils::ptr toTendrils(bp::dict d, int tt)
      {
        bp::list keys = d.keys();
        bp::stl_input_iterator<std::string> begin(keys), end;
        tendrils::ptr ts(new tendrils);

        while (begin != end)
          {
            std::string key = *begin;
            TendrilSpecifications spec = bp::extract<TendrilSpecifications>(d.get(bp::str(key)));
            tendril::ptr tp = spec.toSpec().toTendril(tt);
            ts->declare(key, tp);
            ++begin;

          }
        return ts;

      }
      Vector vts;
    };

    TendrilSpecifications getitem_str(cell::ptr mod, const std::string& key)
    {
      return TendrilSpecifications::Vector(1, TendrilSpecification(mod, key));
    }

    TendrilSpecifications getitem_tuple(cell::ptr mod, bp::tuple keys)
    {
      int end = bp::len(keys);
      TendrilSpecifications l;
      l.vts.reserve(end);
      for (int i = 0; i != end; ++i)
        {
          bp::extract<std::string> se(keys[i]);
          if (se.check())
            l.vts.push_back(TendrilSpecification(mod, se()));
          else
            throw std::runtime_error("All items must be str's");
        }
      return l;
    }

    TendrilSpecifications getitem_list(cell::ptr mod, bp::list keys)
    {
      bp::tuple t(keys);
      return getitem_tuple(mod, t);
    }

    TendrilSpecifications getitem_slice(cell::ptr mod, bp::slice s)
    {

      if (s == bp::slice())
        {
          return TendrilSpecifications::Vector(1, TendrilSpecification(mod, ""));
        }
      else
        {
          throw std::runtime_error("Slice is only valid if its the [:] form...");
        }
    }

    TendrilSpecifications expand(cell::ptr mod, const tendrils& t)
    {
      TendrilSpecifications l;

      BOOST_FOREACH(const tendrils::value_type& pair, t)
        {
          l.vts.push_back(TendrilSpecification(mod, pair.first));
        }
      return l;
    }

    bp::list rshift_spec(TendrilSpecifications& lhs, TendrilSpecifications& rhs)
    {
      bp::list result;
      if (lhs.vts.size() == 1 && lhs.vts.front().key.empty())
        {
          lhs = expand(lhs.vts.front().mod_output, lhs.vts.front().mod_output->outputs);
        }
      if (rhs.vts.size() == 1 && rhs.vts.front().key.empty())
        {
          rhs = expand(rhs.vts.front().mod_input, rhs.vts.front().mod_input->inputs);
        }
      //the spec must be the same size...
      if (lhs.vts.size() != rhs.vts.size())
        {
          std::string msg = boost::str(boost::format("Specification mismatch... len(lhs) != len(rhs) -> %d != %d")
                                       % lhs.vts.size() % rhs.vts.size());
          throw std::runtime_error(msg);
        }
      for (size_t i = 0, end = lhs.vts.size(); i < end; i++)
        {
          TendrilSpecification out = lhs.vts[i], in = rhs.vts[i];
          //check types, this will also assert on not found...
          out.mod_output->outputs[out.key]->compatible_type(*in.mod_input->inputs[in.key]);
          result.append(bp::make_tuple(out.mod_output, out.key, in.mod_input, in.key));
        }
      return result;
    }

    bp::list rshift_spec_tuples(TendrilSpecifications& lhs, bp::tuple& rhs)
    {
      bp::list result;
      bp::stl_input_iterator< TendrilSpecifications& > begin(rhs),end;
      while(begin != end)
        {
          result.extend(rshift_spec(lhs,*begin));
          ++begin;
        }
      return result;
    }

    void wrapModule()
    {
      //use private names so that python people know these are internal
      bp::class_<cell, boost::shared_ptr<cell>, boost::noncopyable>("_module_cpp", bp::no_init);

      bp::class_<cellwrap, boost::shared_ptr<cellwrap>, boost::noncopyable> m_base("_module_base" /*bp::no_init*/);
      m_base.def("declare_params", &cell::declare_params);
      m_base.def("declare_io", ((void(cell::*)()) &cell::declare_io));
      m_base.def("configure", ((void(cell::*)()) &cell::configure));
      m_base.def("process", (void(cell::*)()) &cell::process);

      m_base.add_property("inputs", make_function(&inputs, bp::return_internal_reference<>()));
      m_base.add_property("outputs", make_function(outputs, bp::return_internal_reference<>()));
      m_base.add_property("params", make_function(params, bp::return_internal_reference<>()));
      m_base.def("type", &cell::type);
      m_base.def("name", (std::string(cell::*)() const) &cell::name);
      m_base.def("doc", &cellwrap::doc);
      m_base.def("short_doc",(std::string(cell::*)() const) &cell::short_doc);
      m_base.def("gen_doc", &cell::gen_doc);
      m_base.def("__getitem__", getitem_str);
      m_base.def("__getitem__", getitem_tuple);
      m_base.def("__getitem__", getitem_list);
      m_base.def("__getitem__", getitem_slice);

      bp::class_<TendrilSpecification> ts("TendrilSpecification");
      ts.def_readwrite("module_input", &TendrilSpecification::mod_input);
      ts.def_readwrite("module_output", &TendrilSpecification::mod_output);
      ts.def_readwrite("key", &TendrilSpecification::key);

      bp::class_<TendrilSpecifications> vts("TendrilSpecifications", bp::init<bp::list>());
      vts.def("to_tendrils", &TendrilSpecifications::toTendrils);
      vts.staticmethod("to_tendrils");
      vts.def("to_spec", &TendrilSpecifications::toSpec);
      vts.def("__rshift__", rshift_spec);
      vts.def("__rshift__", rshift_spec_tuples);

    }

  }
}

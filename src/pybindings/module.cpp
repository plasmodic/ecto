#include <ecto/ecto.hpp>
#include <ecto/module.hpp>

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
#if 1
#undef SHOW
#define SHOW() do{}while(false)
#endif

    struct modwrap: module, bp::wrapper<module>
    {

      void dispatch_declare_params(tendrils& params)
      {
        SHOW();
        if (bp::override init = this->get_override("declare_params"))
          init(boost::ref(params));
      }

      void dispatch_declare_io(const tendrils&params, tendrils& inputs,
                               tendrils& outputs)
      {
        SHOW();
        if (bp::override declare_io = this->get_override("declare_io"))
          declare_io(boost::ref(params), boost::ref(inputs),
                     boost::ref(outputs));
      }

      void dispatch_configure(tendrils& params, tendrils& inputs,
                              tendrils& outputs)
      {
        SHOW();
        if (bp::override config = this->get_override("configure"))
          config(boost::ref(params));
      }

      ReturnCode dispatch_process(const tendrils& inputs, tendrils& outputs)
      {
        SHOW();
        if (bp::override proc = this->get_override("process"))
        {
          proc(boost::ref(inputs), boost::ref(outputs));
        }
        return OK;
      }

      void dispatch_destroy()
      {
        SHOW();
        if (bp::override dest = this->get_override("destroy"))
          dest();
      }

      std::string dispatch_name() const
      {
        SHOW();
        bp::reference_existing_object::apply<modwrap*>::type converter;
        PyObject* obj = converter(this);
        bp::object real_obj = bp::object(bp::handle<>(obj));
        bp::object n = real_obj.attr("__class__").attr("__name__");
        std::string nm = bp::extract<std::string>(n);
        return nm;
      }

      static std::string doc(modwrap* mod)
      {
        SHOW();
        bp::reference_existing_object::apply<modwrap*>::type converter;
        PyObject* obj = converter(mod);
        bp::object real_obj = bp::object(bp::handle<>(obj));
        bp::object n = real_obj.attr("__class__").attr("__doc__");
        bp::extract<std::string> get_str(n);
        if (get_str.check())
          return get_str();
        return "No Doc str.";
      }
    };

    const tendrils& inputs(module& mod)
    {
      return mod.inputs;
    }
    tendrils& outputs(module& mod)
    {
      return mod.outputs;
    }
    tendrils& params(module& mod)
    {
      return mod.parameters;
    }

    struct TendrilSpecification
    {
      module::ptr mod;
      bp::tuple keys;
      std::vector<std::string> keys_str;
      TendrilSpecification()
      {
      }
      TendrilSpecification(module::ptr mod, bp::tuple keys) :
          mod(mod), keys(keys)
      {
        int size = bp::len(keys);
        for (int i = 0; i < size; i++)
        {
          bp::object o = keys[i];
          bp::extract<std::string> e(o);
          if (e.check())
          {
            std::string key = e();
            if (mod->inputs.find(key) == mod->inputs.end()
                && mod->outputs.find(key) == mod->outputs.end())
            {
              throw std::runtime_error(
                  "The module does not contain any inputs or outputs by the given name: "
                      + key);
            }
            keys_str.push_back(e());
          }
          else
            throw std::runtime_error("The tuple must contain only str types!.");
        }
      }
      bp::str __str__()
      {
        bp::str str = bp::str(mod->name());
        str += ",[";
        int size = bp::len(keys);
        for (int i = 0; i < size; i++)
        {
          str += "'";
          str += bp::str(keys[i]);
          str += "'";
          if (i < size - 1)
            str += ',';
        }
        str += "]";

        return str;
      }
    };

    TendrilSpecification getitem_str(module::ptr mod, bp::str key)
    {
      return TendrilSpecification(mod, bp::make_tuple(key));
    }

    TendrilSpecification getitem_tuple(module::ptr mod, bp::tuple keys)
    {
      return TendrilSpecification(mod, keys);
    }

    TendrilSpecification getitem_list(module::ptr mod, bp::list keys)
    {
      bp::tuple t(keys);
      return TendrilSpecification(mod, t);
    }

    TendrilSpecification getitem_slice(module::ptr mod, bp::slice s)
    {

      if (s == bp::slice())
      {
        return TendrilSpecification(mod, bp::tuple());
      }
      else
      {
        throw std::runtime_error("Slice is only valid if its the [:] form...");
      }
    }
    TendrilSpecification expand(module::ptr mod, const tendrils& t)
    {
      bp::list x;
      BOOST_FOREACH(const tendrils::value_type& pair, t)
      {
        x.append(bp::str(pair.first));
      }
      return TendrilSpecification(mod, bp::tuple(x));
    }
    bp::list rshift_spec(TendrilSpecification lhs,
                         TendrilSpecification rhs)
    {
      bp::list result;
      if(lhs.keys_str.size() == 0)
      {
        lhs = expand(lhs.mod,lhs.mod->outputs);
      }
      if(rhs.keys_str.size() == 0)
      {
        rhs = expand(rhs.mod,rhs.mod->inputs);
      }
      //the spec must be the same size...
      if (lhs.keys_str.size() != rhs.keys_str.size())
      {
        std::string msg = boost::str(
            boost::format(
                "Specification mismatch... len(lhs) != len(rhs) -> %d != %d")
                % lhs.keys_str.size() % rhs.keys_str.size());
        throw std::runtime_error(msg);
      }
      for (size_t i = 0, end = lhs.keys_str.size(); i < end; i++)
      {
        //check types, this will also assert on not found...
        lhs.mod->outputs.at(lhs.keys_str[i])->compatible_type(
            *rhs.mod->inputs.at(rhs.keys_str[i]));
        result.append(
            bp::make_tuple(lhs.mod, lhs.keys_str[i], rhs.mod, rhs.keys_str[i]));
      }
      return result;
    }

    void wrapModule()
    {
      //use private names so that python people know these are internal
      bp::class_<module, boost::shared_ptr<module>, boost::noncopyable>(
          "_module_cpp", bp::no_init);

      bp::class_<modwrap, boost::shared_ptr<modwrap>, boost::noncopyable> m_base(
          "_module_base" /*bp::no_init*/);
      m_base.def("declare_params", &module::declare_params);
      m_base.def("declare_io", ((void(module::*)()) &module::declare_io));m_base
      .def("configure", ((void(module::*)()) &module::configure));m_base
      .def("process", (void(module::*)()) &module::process);m_base
      .def("destroy", &module::destroy);

      m_base.add_property(
          "inputs", make_function(&inputs, bp::return_internal_reference<>()));
      m_base.add_property(
          "outputs", make_function(outputs, bp::return_internal_reference<>()));
      m_base.add_property(
          "params", make_function(params, bp::return_internal_reference<>()));
      m_base.def("type", &module::type);
      m_base.def("name", (std::string(module::*)() const) &module::name);m_base
      .def("doc", &modwrap::doc);
      m_base.def("gen_doc", &module::gen_doc);
      m_base.def("__getitem__", getitem_str);
      m_base.def("__getitem__", getitem_tuple);
      m_base.def("__getitem__", getitem_list);
      m_base.def("__getitem__", getitem_slice);

      bp::class_<TendrilSpecification> ts("TendrilSpecification");
      ts.def_readwrite("module", &TendrilSpecification::mod);
      ts.def_readwrite("keys", &TendrilSpecification::keys);
      ts.def("__rshift__", rshift_spec);
      ts.def("__str__", &TendrilSpecification::__str__);

    }

  }
}

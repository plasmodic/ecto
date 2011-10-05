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
#include "tendril_spec.hpp"

namespace ecto
{
  namespace py
  {

    struct cellwrap: cell, bp::wrapper<cell>
    {
      cellwrap():initialized_(false){}

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

      void dispatch_configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        if (bp::override config = this->get_override("configure"))
          config(boost::ref(params));
      }

      struct YouveBeenServed
      {
        void operator()(const tendrils::value_type& t)
        {
          t.second->notify();
        }
      };

      ReturnCode dispatch_process(const tendrils& inputs, const tendrils& outputs)
      {
        int value = OK;
        std::for_each(inputs.begin(),inputs.end(), YouveBeenServed());
        if (bp::override proc = this->get_override("process"))
          {
            value = proc(boost::ref(inputs), boost::ref(outputs));
          }
        std::for_each(outputs.begin(),outputs.end(),YouveBeenServed());
        return ReturnCode(value);
      }

      bool init()
      {
        bool initialized = initialized_;
        initialized_ = false;
        return initialized;
      }

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

      cell::ptr dispatch_clone() const
      {
        throw std::logic_error("Clone is not implemented!");
        return cell::ptr();
      }
      bool initialized_;
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

    void wrapModule()
    {
      //use private names so that python people know these are internal
      bp::class_<cell, boost::shared_ptr<cell>, boost::noncopyable>("_cell_cpp", bp::no_init)
        .def("type", &cell::type)
        .def("configure", ((void(cell::*)()) &cell::configure))
        ;

      bp::class_<cellwrap, boost::shared_ptr<cellwrap>, boost::noncopyable> ("_cell_base" /*bp::no_init*/)
        .def("_set_strand", &cell::set_strand)
        .def("_reset_strand", &cell::reset_strand)
        .def("construct", &inspect_impl)
        .def("declare_params", &cell::declare_params)
        .def("declare_io", ((void(cell::*)()) &cell::declare_io))
        .def("configure", ((void(cell::*)()) &cell::configure))
        .def("process", (void(cell::*)()) &cell::process)

        .add_property("inputs", make_function(&inputs, bp::return_internal_reference<>()))
        .add_property("outputs", make_function(outputs, bp::return_internal_reference<>()))
        .add_property("params", make_function(params, bp::return_internal_reference<>()))
        .def("typename", &cell::type)
        .def("name",(((std::string(cell::*)() const) &cell::name)))
        .def("name",(((void(cell::*)(const std::string&)) &cell::name)))
        
        .def("doc", &cellwrap::doc)
        .def("short_doc",(std::string(cell::*)() const) &cell::short_doc)
        .def("gen_doc", &cell::gen_doc)
        .def("__getitem__", getitem_str)
        .def("__getitem__", getitem_tuple)
        .def("__getitem__", getitem_list)
        .def("__getitem__", getitem_slice)
        ;

      bp::def("__getitem_str__", getitem_str);
      bp::def("__getitem_slice__", getitem_slice);
      bp::def("__getitem_tuple__", getitem_tuple);
      bp::def("__getitem_list__", getitem_list);

      bp::class_<TendrilSpecification>("TendrilSpecification")
        .def_readwrite("module_input", &TendrilSpecification::mod_input)
        .def_readwrite("module_output", &TendrilSpecification::mod_output)
        .def_readwrite("key", &TendrilSpecification::key)
        .def("to_tendril",&TendrilSpecification::toTendril)
        ;

      bp::class_<TendrilSpecifications>("TendrilSpecifications", bp::init<bp::list>())
        .def("to_tendrils", &TendrilSpecifications::toTendrils)
        .staticmethod("to_tendrils")
        .def("to_spec", &TendrilSpecifications::toSpec)
        .def("__rshift__", rshift_spec)
        .def("__rshift__", rshift_spec_tuples)
        ;

      bp::enum_<tendril_type>("tendril_type")
        .value("INPUT",INPUT)
        .value("OUTPUT",OUTPUT)
        .value("PARAMETER",PARAMETER)
        .export_values()
        ;

    }
  }

  void inspect_impl(ecto::cell::ptr m, const boost::python::tuple& args, const boost::python::dict& kwargs)
  {
    
    if (bp::len(args) > 1)
      throw std::runtime_error("Only one non-keyword argument allowed, this will specify instance name");

    if (bp::len(args) == 0)
      {
        // generate default name == type
        m->name(m->type());
      }
    else 
      {
        bp::extract<std::string> e(args[0]);
        if (! e.check())
          throw std::runtime_error("Non-keyword argument (instance name) not convertible to string.");
        m->name(e());
      }
    m->declare_params();

    bp::list l = kwargs.items();
    for (int j = 0; j < bp::len(l); ++j)
      {
        bp::object key = l[j][0];
        bp::object value = l[j][1];
        std::string keystring = bp::extract<std::string>(key);
        if (keystring == "strand")
          {
            ecto::strand s = bp::extract<ecto::strand>(value);
            m->strand_ = s;
          }
        else 
          {
            tendril::ptr tp = m->parameters[keystring];
            try{
              *tp << value;
            }catch(ecto::except::TypeMismatch& e)
              {
                e << except::tendril_key(keystring);
                e << except::cell_name(m->name());
                throw;
              }
            tp->user_supplied(true);
            tp->dirty(true);
          }
      }
    m->declare_io();
  }
}

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

    void wrapModule()
    {
      //use private names so that python people know these are internal
      bp::class_<cell, boost::shared_ptr<cell>, boost::noncopyable>("_cell_cpp", bp::no_init);

      bp::class_<cellwrap, boost::shared_ptr<cellwrap>, boost::noncopyable> m_base("_cell_base" /*bp::no_init*/);
      m_base.def("declare_params", &cell::declare_params);
      m_base.def("declare_io", ((void(cell::*)()) &cell::declare_io));
      m_base.def("configure", ((void(cell::*)()) &cell::configure));
      m_base.def("process", (void(cell::*)()) &cell::process);

      m_base.add_property("inputs", make_function(&inputs, bp::return_internal_reference<>()));
      m_base.add_property("outputs", make_function(outputs, bp::return_internal_reference<>()));
      m_base.add_property("params", make_function(params, bp::return_internal_reference<>()));
      m_base.def("type", &cell::type);
      m_base.def("name",(((std::string(cell::*)() const) &cell::name)));
      m_base.def("name",(((void(cell::*)(const std::string&)) &cell::name)));

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
      ts.def("to_tendril",&TendrilSpecification::toTendril);
      bp::class_<TendrilSpecifications> vts("TendrilSpecifications", bp::init<bp::list>());
      vts.def("to_tendrils", &TendrilSpecifications::toTendrils);
      vts.staticmethod("to_tendrils");
      vts.def("to_spec", &TendrilSpecifications::toSpec);
      vts.def("__rshift__", rshift_spec);
      vts.def("__rshift__", rshift_spec_tuples);

      bp::enum_<tendril_type>("tendril_type")
          .value("INPUT",INPUT)
          .value("OUTPUT",OUTPUT)
          .value("PARAMETER",PARAMETER)
          .export_values()
          ;

    }

  }
}

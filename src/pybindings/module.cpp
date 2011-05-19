#include <ecto/module.hpp>
#include <ecto/ecto.hpp>

#include <boost/python.hpp>
#include <boost/python/raw_function.hpp>
#include <ecto/python/std_map_indexing_suite.hpp>

#include <ecto/python/raw_constructor.hpp>
#include <ecto/python/repr.hpp>

namespace bp = boost::python;

namespace ecto
{
namespace py
{

struct modwrap: module, bp::wrapper<module>
{
  modwrap() :
    module(/*&modwrap::Initialize,*/
    boost::bind(&modwrap::configure, this, _1, _2, _3), boost::bind(&modwrap::process, this, _1, _2, _3))
  {

  }

  static
  void Initialize(ecto::tendrils& parameters)
  {

  }

  void process(const ecto::tendrils& parameters, const ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    if (bp::override process = this->get_override("process"))
      process();
    else
      throw std::logic_error("process is not implemented it seems");
  }

  void configure(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    if (bp::override config = this->get_override("configure"))
      config();
    else
      throw std::logic_error("configure is not implemented it seems");
  }

  std::string name()
  {
    bp::reference_existing_object::apply<modwrap*>::type converter;
    PyObject* obj = converter(this);
    bp::object real_obj = bp::object(bp::handle<>(obj));
    bp::object n = real_obj.attr("__class__").attr("__name__");
    std::string nm = bp::extract<std::string>(n);
    return nm;
  }

  static std::string doc(modwrap* mod)
  {
    bp::reference_existing_object::apply<modwrap*>::type converter;
    PyObject* obj = converter(mod);
    bp::object real_obj = bp::object(bp::handle<>(obj));
    bp::object n = real_obj.attr("__class__").attr("__doc__");
    std::string nm = bp::extract<std::string>(n);
    return nm;
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
void wrapModule()
{
  //use private names so that python people know these are internal
  bp::class_<module, boost::shared_ptr<module>, boost::noncopyable>("_module_cpp", bp::no_init);

  bp::class_<modwrap, boost::shared_ptr<modwrap>, boost::noncopyable> m_base("_module_base" /*bp::no_init*/);
  m_base.def("connect", &module::connect);
  m_base.def("process", bp::pure_virtual((void(module::*)()) &module::process));
  m_base.def("configure", bp::pure_virtual((void(module::*)()) &module::configure));
  m_base.add_property("inputs", make_function(&inputs, bp::return_internal_reference<>()));
  m_base.add_property("outputs", make_function(outputs, bp::return_internal_reference<>()));
  m_base.add_property("params", make_function(params, bp::return_internal_reference<>()));
  m_base.add_property("name",make_function(params, bp::return_internal_reference<>()));
  m_base.def("doc", &modwrap::doc);
}

}
}


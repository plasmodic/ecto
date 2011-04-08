#include <ecto/module.hpp>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

namespace bp = boost::python;

namespace ecto
{
  namespace py
  {

    struct modwrap : module, bp::wrapper<module>
    {
      void Process()
      {
        this->get_override("Process")();
      }

      void Config()
      {
        this->get_override("Config")();
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
      static std::string nameBp(bp::object o)
      {
        bp::object n = o["__class__"]["__name__"];
        modwrap& m = bp::extract<modwrap&>(o);
        m.name_ = bp::extract<std::string>(n);
        return m.name_;
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
      std::string name_;
    };

    void setTendril(tendrils& t, const std::string& name, const std::string& doc, bp::object o)
    {
      t.set(name, doc, o);
    }

    bp::object getTendril(tendrils& t, const std::string& name)
    {
      return t[name].extract();
    }

    void wrapModule()
    {
      bp::class_<tendrils> tendrils_("tendrils");
      tendrils_.def(bp::map_indexing_suite<tendrils, false>());
      tendrils_.def("set", setTendril);
      tendrils_.def("get", getTendril);
      bp::class_<module, boost::shared_ptr<module>, boost::noncopyable>("module");
      bp::class_<modwrap, boost::noncopyable> mw("module");
      mw.def("connect", &module::connect);
      mw.def("Process", bp::pure_virtual(&module::Process));
      mw.def("Config", bp::pure_virtual(&module::Config));
      mw.def_readwrite("inputs", &module::inputs);
      mw.def_readwrite("outputs", &module::outputs);
      mw.def_readwrite("params", &module::params);
      mw.def("Name", &module::name);
      mw.def("Doc", &modwrap::doc);
    }

  }
}


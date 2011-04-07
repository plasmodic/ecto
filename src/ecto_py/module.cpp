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

      static std::string name()
      {
        return "module";
      }
      static std::string doc()
      {
        return "doc";
      }
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
      mw.def("Name", &modwrap::name);
      mw.staticmethod("Name");
      mw.def("Doc", &modwrap::doc);
      mw.staticmethod("Doc");
    }

  }
}


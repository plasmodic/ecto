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
    void Process() {
      //std::cout << "dispatching Process...\n";
      this->get_override("Process")();
    }

    void Config() {
      //std::cout << "dispatching Config...\n";
      this->get_override("Config")();
    }

    void setIn(const std::string& name, const std::string& doc, 
	       bp::object obj)
    {
      this->module::setIn<bp::object>(name, doc, obj);
    }

    void setOut(const std::string& name, const std::string& doc, 
		bp::object obj)
    {
      this->module::setOut<bp::object>(name, doc, obj);
    }

    bp::object getIn(const std::string& name)
    {
      return this->module::getIn<bp::object>(name);
    }

    bp::object getParam(const std::string& name)
    {
      return this->module::getParam<bp::object>(name);
    }

    void put(const std::string& name, bp::object obj)
    {
      this->module::getOut<bp::object>(name) = obj;
    }
    static std::string name()
    {
     return "module";
    }
    static std::string doc()
    {
      //this->get_override("Doc");
      return "doc";
    }
  };

void wrapModule(){

  bp::class_<tendrils>("tendrils")
    .def(bp::map_indexing_suite<tendrils,false>())
        ;

  bp::class_<modwrap, boost::shared_ptr<module>,boost::noncopyable>("module")
      .def("connect", &module::connect)
      .def("Process", &module::Process)
      .def("Config", &module::Config)
      .def("Name",&modwrap::name)
      .staticmethod("Name")
      .def("Doc",&modwrap::doc)
      .staticmethod("Doc")
      .def("setIn", &modwrap::setIn)
      .def("setOut", &modwrap::setOut)
      .def("getIn", &modwrap::getIn)
      .def("getParam", &modwrap::getParam)
      .def("put", &modwrap::put)
      .def_readwrite("inputs", &module::inputs)
      .def_readwrite("outputs", &module::outputs)
      .def_readwrite("params", &module::params)
    ;
//  void a_map_indexing_suite(); // moved to a_map_indexing_suite.cpp to
//     a_map_indexing_suite();
  /*
  bp::class_<module, boost::noncopyable>("module")
    .def_readwrite("inputs", &module::inputs)
    .def_readwrite("outputs", &module::outputs)
    .def_readwrite("params", &module::params)
    .def("connect", &module::connect)
    .def("process", &module::Process)
    ;
  */
}

}
}


#include <ecto/module.hpp>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

namespace bp = boost::python;

namespace ecto
{
namespace py
{

void wrapModule(){

  bp::class_<module::connections_t>("connections")
    .def(bp::map_indexing_suite<module::connections_t>())
    ;

  bp::class_<module, boost::noncopyable>("module")
    .def_readwrite("inputs", &module::inputs)
    .def_readwrite("outputs", &module::outputs)
    .def_readwrite("params", &module::params)
    .def("connect", &module::connect)
    .def("process", &module::Process)
    ;
}

}
}


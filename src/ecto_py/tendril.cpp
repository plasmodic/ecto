#include <ecto/tendril.hpp>

#include <boost/python.hpp>

namespace bp = boost::python;

namespace ecto
{
namespace py
{

void wrapConnection(){
  bp::class_<tendril>("tendril", bp::no_init)
    .def("type_name", &tendril::type_name)
    .def("value", &tendril::value)
    .def("connect", &tendril::connect)
    .def("name",&tendril::name, "Give the name of this connection.")
    .def("doc",&tendril::doc)
    .def("get",&tendril::extractFromPython)
    .def("set",&tendril::setFromPython)
    ;
}

}
}


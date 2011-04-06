#include <ecto/tendril.hpp>

#include <boost/python.hpp>

namespace bp = boost::python;

namespace ecto
{
namespace py
{

void wrapConnection(){
  bp::class_<tendril>("tendril")
    .def("type_name", &tendril::type_name)
    .def("connect", &tendril::connect)
    .def("doc",&tendril::doc)
    .def("get",&tendril::extract)
    .def("set",(void(tendril::*)(bp::object)) &tendril::set)
    ;
}

}
}


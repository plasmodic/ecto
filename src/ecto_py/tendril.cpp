#include <ecto/tendril.hpp>

#include <boost/python.hpp>

namespace bp = boost::python;

namespace ecto
{
namespace py
{

void wrapConnection(){
  bp::class_<tendril>("tendril")
    .def("connect", &tendril::connect)
    .add_property("doc",&tendril::doc,&tendril::setDoc)
    .add_property("type_name", &tendril::type_name)
    .add_property("val", &tendril::extract,(void(tendril::*)(bp::object)) &tendril::set)
    .def("get",&tendril::extract)
    .def("set",(void(tendril::*)(bp::object)) &tendril::set)
    ;
}

}
}


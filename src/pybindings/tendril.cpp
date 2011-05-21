#include <ecto/tendril.hpp>

#include <boost/python.hpp>

namespace bp = boost::python;

namespace ecto
{
namespace py
{

boost::shared_ptr<tendril> tendril_ctr()
{
  return boost::shared_ptr<tendril>(new tendril(bp::object(),"A pythonic tendril."));
}

void wrapConnection(){
  bp::class_<tendril,boost::shared_ptr<tendril> >("Tendril")
    .def("__init__", bp::make_constructor(tendril_ctr))
    .add_property("doc",&tendril::doc,&tendril::setDoc)
    .add_property("type_name", &tendril::type_name)
    .add_property("val", &tendril::extract,(void(tendril::*)(bp::object)) &tendril::set)
    .def("get",&tendril::extract)
    .def("set",(void(tendril::*)(bp::object)) &tendril::set)
    ;
}

}
}


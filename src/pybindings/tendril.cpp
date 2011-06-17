#include <ecto/tendril.hpp>

#include <boost/python.hpp>

namespace bp = boost::python;

namespace ecto
{
namespace py
{

tendril::ptr tendril_ctr()
{
  return boost::shared_ptr<tendril>(new tendril(bp::object(),"A pythonic tendril."));
}

std::string tendril_type_name(tendril::ptr t)
{
  return t->type_name();
}

std::string tendril_doc(tendril::ptr t)
{
  return t->doc();
}

void tendril_set_doc(tendril::ptr t, const std::string& doc)
{
  return t->setDoc(doc);
}

bp::object tendril_get_val(tendril::ptr t)
{
  return t->extract();
}

void tendril_set_val(tendril::ptr t, bp::object val)
{
  t->set(val);
}

void wrapConnection(){
  bp::class_<tendril,boost::shared_ptr<tendril> >("Tendril")
    .def("__init__", bp::make_constructor(tendril_ctr))
    .add_property("doc",tendril_doc,&tendril::setDoc)
    .add_property("type_name",tendril_type_name )
    .add_property("val", tendril_get_val,tendril_set_val)
    .def("get",tendril_get_val)
    .def("set",tendril_set_val)
    ;
}

}
}


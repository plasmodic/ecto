#include <boost/python.hpp>

#include <ecto/tendril.hpp>

#include <boost/foreach.hpp>


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
  return t->set_doc(doc);
}

bp::object tendril_get_val(tendril::ptr t)
{
  bp::object o;
  t >> o;
  return o;
}

void tendril_set_val(tendril::ptr t, bp::object val)
{
  t << val;
  t->dirty(true);
  t->user_supplied(true);
}
void tendril_copy_val(tendril::ptr t, tendril::ptr tv)
{
  t << *tv;
}
bool tendril_user_supplied(tendril::ptr t)
{
  return t->user_supplied();
}

bool tendril_has_default(tendril::ptr t)
{
  return t->has_default();
}

bool tendril_dirty(tendril::ptr t)
{
  return t->dirty();
}

bool tendril_required(tendril::ptr t)
{
  return t->required();
}

void wrapConnection(){
  bp::class_<tendril,boost::shared_ptr<tendril> > Tendril_("Tendril", 
      "The Tendril is the slendor winding organ of ecto.\n"
      "It is a type erasing holder with meta data that enable introspection.");
    Tendril_.def("__init__", bp::make_constructor(tendril_ctr));
    Tendril_.add_property("doc",tendril_doc,&tendril::set_doc, "A doc string that describes the purpose of this tendril.");
    Tendril_.add_property("type_name",tendril_type_name, "The type of the value held by the tendril." );
    Tendril_.add_property("val", tendril_get_val,tendril_set_val, "The value held by the tendril.\n"
        "It requires boost::python bindings to be accessible from python.\n"
        "If none are available it will be None.");
    Tendril_.add_property("user_supplied",tendril_user_supplied, "Has the value been set by the user?");
    Tendril_.add_property("has_default",tendril_has_default, "Does the tendril have an explicit default value?\n"
        "Remember that the implicit default is always the default constructed type.");
    Tendril_.add_property("required",tendril_required, "Is this tendril required to be connected?");
    Tendril_.add_property("dirty",tendril_dirty, "Has the tendril changed since the last time?");
    Tendril_.def("get",tendril_get_val, "Gets the python value of the object.\n"
    "May be None if python bindings for the type held do not have boost::python bindings available from the current scope."
    );
    Tendril_.def("set",tendril_set_val, "Assuming the value held by the tendril has boost::python bindings,\nthis will copy the value of the given python object into the value held by the tendril.");
    Tendril_.def("copy_value",tendril_copy_val, "Copy from one tendril to the other.");
    Tendril_.def("notify",&tendril::notify, "Force updates.");
}
}
}


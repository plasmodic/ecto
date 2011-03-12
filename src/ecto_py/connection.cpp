#include <ecto/connection.hpp>

#include <boost/python.hpp>

namespace bp = boost::python;

namespace ecto
{
namespace py
{

void wrapConnection(){
  bp::class_<connection>("Connection", bp::no_init)
    .def("type_name", &connection::type_name)
    .def("value", &connection::value)
    .def("connect", &connection::connect)
    .def("name",&connection::name, "Give the name of this connection.")
    .def("doc",&connection::doc)
    ;
}

}
}


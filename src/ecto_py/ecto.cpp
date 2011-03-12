#include <ecto/ecto.hpp>
#include <ecto/util.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

#include <map>
#include <set>
#include <sstream>

namespace bp = boost::python;

BOOST_PYTHON_MODULE(ecto)
{
  using namespace ecto;

  bp::class_<connection>("Connection", bp::no_init)
    .def("type_name", &connection::type_name)
    .def("value", &connection::value)
    .def("connect", &connection::connect)
    .def("name",&connection::name, "Give the name of this connection.")
    .def("doc",&connection::doc)
    ;

  bp::class_<plasm,boost::noncopyable>("Plasm")
    .def("connect", &plasm::connect)
    .def("markDirty", &plasm::markDirty)
    .def("go", &plasm::go)
    .def("viz",&plasm::viz)
    ;

  bp::class_<module::connections_t>("Connections")
    .def(bp::map_indexing_suite<module::connections_t>())
    ;

  bp::class_<module, boost::noncopyable>("Module")
    .def_readwrite("inputs", &module::inputs)
    .def_readwrite("outputs", &module::outputs)
    .def("connect", &module::connect)
    .def("Process", &module::Process)
    ;
}




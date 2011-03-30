#include <ecto/plasm.hpp>
#include <ecto/module.hpp>
//boost python junk
//#include "ecto_split.h"

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

namespace bp = boost::python;

namespace ecto
{
namespace py
{

using bp::arg;
std::string wrapViz(const ecto::plasm& p)
{
  return p.viz();
}
void wrapPlasm(){
//  bp::class_<edge>("Edge")
//    .def_readwrite("downstream",&edge::downstream)
//    .def_readwrite("upstream",&edge::upstream)
//    ;
//  bp::class_<plasm::map_t>("EdgeMap")
//    .def(bp::map_indexing_suite<plasm::map_t>())
//    ;
//  bp::class_<edge::us::modules>("Upstream")
//    .def(bp::map_indexing_suite<edge::us::modules>())
//    ;
//  bp::class_<edge::ds::modules>("Downstream")
//    .def(bp::map_indexing_suite<edge::ds::modules>())
//    ;
//  bp::class_<edge::ds::module_set>("ModuleSet")
//      .def(bp::set_indexing_suite<edge::ds::module_set>())
//      ;

  bp::class_<plasm,boost::noncopyable>("Plasm")
    .def("connect", &plasm::connect, (arg("from_module"), arg("output_name"),
				      arg("to_module"),   arg("intput_name")))
    .def("markDirty", &plasm::markDirty)
    .def("go", &plasm::go)
    .def("viz",wrapViz)
    ;
}

}
}


#include <ecto/plasm.hpp>
#include <ecto/module.hpp>
//boost python junk
//#include "ecto_split.h"

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

namespace bp = boost::python;
namespace boost { namespace python {

    // Forward declaration
    template <class Container, bool NoProxy, class DerivedPolicies>
    class set_indexing_suite;

    namespace detail
    {
        template <class Container, bool NoProxy>
        class final_set_derived_policies
            : public set_indexing_suite<Container,
                NoProxy, final_set_derived_policies<Container, NoProxy> > {};
    }

template <
    class Container,
    bool NoProxy = false,
    class DerivedPolicies
        = detail::final_set_derived_policies<Container, NoProxy> >
class set_indexing_suite
    : public indexing_suite<
        Container
      , DerivedPolicies
      , false
      , true
      , typename Container::value_type
      , typename Container::key_type
      , typename Container::key_type
    >
{
public:

    typedef typename Container::value_type value_type;
    typedef typename Container::value_type data_type;
    typedef typename Container::key_type key_type;
    typedef typename Container::key_type index_type;
    typedef typename Container::size_type size_type;
    typedef typename Container::difference_type difference_type;

    template <class Class>
    static void
    extension_def(Class& cl)
    {
        //  Wrap the map's element (value_type)
        std::string elem_name = "set_indexing_suite_";
        object class_name(cl.attr("__name__"));
        extract<std::string> class_name_extractor(class_name);
        elem_name += class_name_extractor();
        elem_name += "_entry";

        class_<value_type>(elem_name.c_str())
            .def("__repr__", &DerivedPolicies::print_elem)
            .def("data", &DerivedPolicies::get_data)
            .def("key", &DerivedPolicies::get_data)
        ;
    }

    static object
    print_elem(typename Container::value_type const& e)
    {
        return "(%s)" % python::make_tuple(e);
    }

    static
    typename Container::value_type
    get_data(typename Container::value_type& e)
    {
        return e;
    }

    static typename Container::key_type
    get_key(typename Container::value_type& e)
    {
        return e;
    }

    static data_type
    get_item(Container& container, index_type i_)
    {
        typename Container::iterator i = container.find(i_);
        if (i == container.end())
        {
            PyErr_SetString(PyExc_KeyError, "Invalid key");
            throw_error_already_set();
        }
        return *i;
    }

    static void
    set_item(Container& container, index_type i, data_type const& v)
    {
        container.insert(v);
    }

    static void
    delete_item(Container& container, index_type i)
    {
        container.erase(i);
    }

    static size_t
    size(Container& container)
    {
        return container.size();
    }

    static bool
    contains(Container& container, key_type const& key)
    {
        return container.find(key) != container.end();
    }

    static bool
    compare_index(Container& container, index_type a, index_type b)
    {
        return container.key_comp()(a, b);
    }

    static index_type
    convert_index(Container& /*container*/, PyObject* i_)
    {
        extract<key_type const&> i(i_);
        if (i.check())
        {
            return i();
        }
        else
        {
            extract<key_type> i(i_);
            if (i.check())
                return i();
        }

        PyErr_SetString(PyExc_TypeError, "Invalid index type");
        throw_error_already_set();
        return index_type();
    }
};
}
}
namespace ecto
{
namespace py
{

void wrapPlasm(){
  bp::class_<edge>("Edge")
    .def_readwrite("downstream",&edge::downstream)
    .def_readwrite("upstream",&edge::upstream)
    ;
  bp::class_<plasm::map_t>("EdgeMap")
    .def(bp::map_indexing_suite<plasm::map_t>())
    ;
  bp::class_<edge::us::modules>("Upstream")
    .def(bp::map_indexing_suite<edge::us::modules>())
    ;
  bp::class_<edge::ds::modules>("Downstream")
    .def(bp::map_indexing_suite<edge::ds::modules>())
    ;
  bp::class_<edge::ds::module_set>("ModuleSet")
      .def(bp::set_indexing_suite<edge::ds::module_set>())
      ;
  bp::class_<plasm,boost::noncopyable>("Plasm")
    .def_readwrite("edges",&plasm::edge_map)
    .def("connect", &plasm::connect)
    .def("markDirty", &plasm::markDirty)
    .def("go", &plasm::go)
    .def("viz",&plasm::viz)
    ;
}

}
}


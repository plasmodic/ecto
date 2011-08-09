#include <boost/python.hpp>

#include <ecto/ecto.hpp>
#include <ecto/cell.hpp>

#include <ecto/python/std_map_indexing_suite.hpp>
#include <boost/foreach.hpp>

namespace bp = boost::python;
//namespace boost { namespace python {
//
//    //typedef detail::final_std_map_derived_policies<ecto::tendrils, false> details;
//    // Forward declaration
//    void std_map_indexing_suite<ecto::tendrils,false>::set_item(ecto::tendrils& container, ecto::tendrils::key_type i, ecto::tendrils::value_type::second_type const& v)
//    {
//        throw std::logic_error("This don't work with tendrils");
//    }
//}
//}
namespace ecto
{
  namespace py
  {
    namespace
    {
      void declareTendril(tendrils& t, const std::string& name, 
                      const std::string& doc, bp::object o)
      {
        t.declare<bp::object> (name, doc, o);
      }

      bp::list tendril_members(const tendrils& t)
      {
        bp::list l;
        for (tendrils::const_iterator iter = t.begin(), end = t.end(); iter != end; ++iter)
          l.append(iter->first);
        return l;
      }

      bp::object getTendril(tendrils& t, const std::string& name)
      {
        bp::object o;
        tendril::ptr tp = t[name];
        *tp >> o;
        return o;
      }

      std::string strTendril(const tendrils& t)
      {
        std::string s = "tendrils:\n";
        for (tendrils::const_iterator iter = t.begin(), end = t.end(); iter != end; ++iter)
        {
          s += "    " + iter->first + " [" + iter->second->type_name() + "]\n";
        }
        return s;
      }

      bp::object tendril_get(const tendrils& ts, const std::string& name)
      {
        if (name == "__members__")
          return tendril_members(ts);

        const tendril& t = *ts[name];
        bp::object o;
        t >> o;
        return o;
      }

      void tendril_set(tendrils& ts, const std::string& name, bp::object obj)
      {
        tendril::ptr t = ts[name];
        t << obj;
        t->dirty(true);
        t->user_supplied(true);
      }

      void tendrils_notify(tendrils& ts)
      {
        BOOST_FOREACH(tendrils::value_type& val, ts)
        {
          val.second->notify();
        }
      }

      tendril::ptr tendril_at(tendrils& ts, const std::string& name)
      {
        return ts[name];
      }
    }

    void wrapTendrils()
    {
      bp::class_<tendrils, boost::shared_ptr<tendrils>, boost::noncopyable>("Tendrils")
        .def(bp::std_map_indexing_suite<tendrils, false>())
        .def("declare", &declareTendril)
        .def("__str__", &strTendril)
        .def("__getattr__", &tendril_get)
        .def("__setattr__", &tendril_set)
        .def("__getitem__", &tendril_get)
        .def("at",tendril_at)
        .def("notify",tendrils_notify)
        ;
    }
  }
}

// Original by Matthew Scouten and Hans Meine
// copied from http://mail.python.org/pipermail/cplusplus-sig/2009-May/014505.html
//
// Modified by Jakob van Santen, 2009-07-26

// This visitor class exposes copy constructors to Python, in particular to the copy module. 
// This should work for any class that defines a (const &) constructor. Usage is similar
// to the indexing suites:
//
// class_<Foo>("Foo")
//   .def(copy_suite<Foo>())
// ;

#ifndef ICETRAY_PYTHON_COPY_SUITE_HPP_INCLUDED
#define ICETRAY_PYTHON_COPY_SUITE_HPP_INCLUDED

#include <boost/python/def_visitor.hpp>

namespace bp = boost::python;

namespace boost { namespace python {

   template <class Copyable>
   class copy_suite : public bp::def_visitor<copy_suite<Copyable > > {
      public:

         template <class Class>
         static void
         visit(Class& cl)
         {
             cl
    		.def("__copy__", &copy_suite<Copyable >::generic__copy__, 
                                 "Make a shallow copy using the copy constructor")
    		.def("__deepcopy__", &copy_suite<Copyable >::generic__deepcopy__, 
                                 "Make a deep copy using the copy constructor")
    		.def(init< const Copyable & >())
               ;
         }

      private:

         template<class T>
         static inline PyObject * managingPyObject(T *p)
         {
             return typename bp::manage_new_object::apply<T *>::type()(p);
         }
         
         static bp::object
         generic__copy__(bp::object copyable)
         {
             Copyable *newCopyable(new Copyable(bp::extract<const Copyable &>(copyable)));
             bp::object result(bp::detail::new_reference(managingPyObject(newCopyable)));
         
             bp::extract<bp::dict>(result.attr("__dict__"))().update(
                 copyable.attr("__dict__"));
         
             return result;
         }
         
         static bp::object
         generic__deepcopy__(bp::object copyable, bp::dict memo)
         {
             bp::object copyMod = bp::import("copy");
             bp::object deepcopy = copyMod.attr("deepcopy");
         
             Copyable *newCopyable(new Copyable(bp::extract<const Copyable &>(copyable)));
         
             bp::object result(bp::detail::new_reference(managingPyObject(newCopyable)));
         
             // HACK: copyableId shall be the same as the result of id(copyable) in Python
             // please tell me that there is a better way! (and which ;-p)
             size_t copyableId = (size_t)(copyable.ptr());
             memo[copyableId] = result;
         
             bp::extract<bp::dict>(result.attr("__dict__"))().update(
                 deepcopy(bp::extract<bp::dict>(copyable.attr("__dict__"))(),memo));
         
             return result;
         }
    }; // class copy_suite

}} // namespace boost::python

#endif // ICETRAY_PYTHON_COPY_SUITE_HPP_INCLUDED

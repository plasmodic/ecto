/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
//do not include this in ecto lib files, only in client modules

//boost python first.
#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>

//ecto includes
#include <ecto/version.hpp>
#include <ecto/abi.hpp>
#include <ecto/cell.hpp>
#include <ecto/util.hpp>
#include <ecto/python/raw_constructor.hpp>
#include <ecto/is_threadsafe.hpp>

#include <iostream>
#include <sstream>

/**
 * \namespace ecto
 * \brief ecto is ...
 *
 * ecto is a plugin architecture / pipeline tool based on boost::python.
 */
namespace ecto
{
  template<typename T>
  boost::shared_ptr<ecto::cell_<T> > inspect(boost::python::tuple args,
                                             boost::python::dict kwargs)
  {
    typedef ecto::cell_<T> cell_t;

    namespace bp = boost::python;

    //SHOW();
    // TDS: fixme is this bare pointer stuff necessary?
    boost::shared_ptr<cell_t> mm(new cell_t());
    ecto::cell * m = mm.get();
    
    if (bp::len(args) > 1)
      throw std::runtime_error("Only one non-keyword argument allowed, this will specify instance name");

    if (bp::len(args) == 0)
      {
        // generate default name == type
        std::string defaultname = boost::str(boost::format("%s") % m->type());
        m->name(defaultname);
      }
    else 
      {
        bp::extract<std::string> e(args[0]);
        if (! e.check())
          throw std::runtime_error("Non-keyword argument (instance name) not convertible to string.");
        m->name(e());
      }
    m->declare_params();

    bp::list l = kwargs.items();
    for (int j = 0; j < bp::len(l); ++j)
      {
        bp::object key = l[j][0];
        bp::object value = l[j][1];
        std::string keystring = bp::extract<std::string>(key);
        if (keystring == "strand")
          {
            ecto::strand s = bp::extract<ecto::strand>(value);
            m->strand_ = s;
          }
        else 
          {
            tendril::ptr tp = m->parameters[keystring];
            try{
              *tp << value;
            }catch(ecto::except::TypeMismatch& e)
            {
              e << std::string("Parameter: " + keystring);
              e << std::string("Cell: " + m->name());
              throw e;
            }
            tp->user_supplied(true);
            tp->dirty(true);
          }
      }
    m->declare_io();
    return mm;
  }

  //this adds the autodoc to the cell. TODO remove python duplication...
  template<typename T>
  std::string cell_doc(std::string doc)
  {
    ecto::cell::ptr c = ecto::inspect_cell<T>();
    std::string defaultname = boost::str(boost::format("%s") % c->type());
    c->name(defaultname);
    return c->gen_doc(doc);
  }

  //
  //  Handle strandization of 
  //
  template <typename T, typename U = typename ecto::detail::is_thread_unsafe<T>::type >
  struct deduce_with_strand
  {
    void operator()(boost::shared_ptr<ecto::cell_<T> > p) const { }
  };

  template <typename T>
  struct deduce_with_strand<T, boost::mpl::true_>
  {
    void operator()(boost::shared_ptr<ecto::cell_<T> > p) const
    {
      p->strand_ = deduce_with_strand::strand_;
    }
    static ecto::strand strand_;
  };

  template <typename T> ecto::strand deduce_with_strand<T, boost::mpl::true_>::strand_;


  template<typename T>
  boost::shared_ptr<ecto::cell_<T> > raw_construct(boost::python::tuple args,
                                                   boost::python::dict kwargs)
  {
    boost::shared_ptr<ecto::cell_<T> > c = inspect<T> (args, kwargs);
    deduce_with_strand<T>()(c); 
    c->verify_params();
    return c;
  }

  /**
   * \brief Takes a user cell, UserModule, that follows the ecto::cell idiom and exposes
   * it to python or other plugin architecture.

   * This should be the preferred method of exposing user
   * modules to the outside world.
   *
   * @tparam UserCell A client cell type that implements the idium of an ecto::cell.
   * @param name The name of the cell, this will be the symbolic name exposed
   *        to python or other plugin systems.
   * @param doc_str A highlevel description of your cell.
   */
  template <typename Impl>
  struct cell_wrapper
  {
    typedef typename 
    boost::python::class_<ecto::cell_<Impl>, 
                          boost::python::bases<cell>,
                          boost::shared_ptr<ecto::cell_<Impl> >, 
                          boost::noncopyable>
    type;
  };

  template <typename Impl>
  typename cell_wrapper<Impl>::type 
  wrap(const char* name, std::string doc_str = "An ecto::cell...")
  {
    typedef ecto::cell_<Impl> cell_t;
    ecto::cell_<Impl>::SHORT_DOC = doc_str;

    typename cell_wrapper<Impl>::type 
      m(name, cell_doc<Impl> (doc_str).c_str());
    m.def("__init__",
          boost::python::raw_constructor(&raw_construct<Impl>));
    m.def("inspect", &inspect<Impl> );
    m.staticmethod("inspect");
    m.def("name", (std::string (cell_t::*)() const) &cell_t::name);
    m.def("type_name", (std::string (cell_t::*)() const) &cell_t::type);
    m.def_readonly("short_doc", &cell_t::SHORT_DOC);
    return m;
  }
}

#define ECTO_DEFINE_MODULE(modname)             \
  ECTO_INSTANTIATE_REGISTRY(modname)            \
  void init_module_##modname##_rest() ;         \
  BOOST_PYTHON_MODULE(modname) {                \
    boost::python::import("ecto");              \
    ECTO_REGISTER(modname);                     \
    init_module_##modname##_rest();             \
  }                                             \
  void init_module_##modname##_rest()

#include <ecto/registry.hpp>


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
//boost python first.
#include <boost/python.hpp>
//boost stuff
#include <boost/shared_ptr.hpp>
//do not include this in ecto lib files, only in client modules
#include <boost/format.hpp>

//ecto includes
#include <ecto/module.hpp>
#include <ecto/tendril.hpp>
#include <ecto/tendrils.hpp>
#include <ecto/plasm.hpp>
#include <ecto/util.hpp>
#include <ecto/python/raw_constructor.hpp>
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
  /**
   * \internal
   */
  template<typename T>
  struct doc
  {
    static std::string doc_str; //!<a doc string for humans to read in python.
    static std::string name; //!< the name for this type.
    //! get the doc
    static std::string getDoc()
    {
      return doc_str;
    }
    //! get the name
    static std::string getName()
    {
      return name;
    }
  };
  template<typename T>
  std::string doc<T>::doc_str;
  template<typename T>
  std::string doc<T>::name;

  template<typename T>
  boost::shared_ptr<ecto::module_<T> > inspect(boost::python::tuple args,
                                               boost::python::dict kwargs)
  {
    typedef ecto::module_<T> module_t;

    namespace bp = boost::python;

    //SHOW();
    boost::shared_ptr<module_t> mm(new module_t());
    ecto::module * m = mm.get();
    
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
          throw std::runtime_error("Non-keyword argument (instance name) not converible to string.");
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
            std::cout << "setting strand " << s.id() << "\n";
            m->strand_ = s;
          }
        else 
          {
            std::string valstring =
              bp::extract<std::string>(value.attr("__repr__")());
            m->parameters.at(keystring)->set(value);
          }
      }
    m->declare_io();
    return mm;
  }

  //this adds the autodoc to the module. TODO remove python duplication...
  template<typename T>
  std::string module_doc(std::string doc)
  {
    ecto::module::ptr m = ecto::inspect_module<T>();
    std::string defaultname = boost::str(boost::format("%s") % m->type());
    m->name(defaultname);
    return m->gen_doc(doc);
  }

  template<typename T>
  boost::shared_ptr<ecto::module_<T> > raw_construct(boost::python::tuple args,
                                                     boost::python::dict kwargs)
  {
    boost::shared_ptr<ecto::module_<T> > m = inspect<T> (args, kwargs);
    return m;
  }

  /**
   * \brief Takes a user module, UserModule, that follows the ecto::module idium and exposes
   * it to python or other plugin architecture.

   * This should be the preferred method of exposing user
   * modules to the outside world.
   *
   * @tparam UserModule A client module type that implements the idium of an ecto::module.
   * @param name The name of the module, this will be the symbolic name exposed
   *        to python or other plugin systems.
   * @param doc_str A highlevel description of your module.
   */
  template<typename UserModule>
  void wrap(const char* name, std::string doc_str = "A module...")
  {
    typedef ecto::module_<UserModule> module_t;
    //SHOW();
    boost::python::class_<module_t, boost::python::bases<module>,
        boost::shared_ptr<module_t>, boost::noncopyable>
        m(name, module_doc<UserModule> (doc_str).c_str());
    typedef doc<UserModule> docT;
    docT::name = name;
    docT::doc_str = doc_str;

    m.def("__init__",
          boost::python::raw_constructor(&raw_construct<UserModule> ));
    m.def("inspect", &inspect<UserModule> );
    m.staticmethod("inspect");
    m.def("doc", &docT::getDoc).staticmethod("doc");
    m.def("name", (std::string (module_t::*)() const) &module_t::name);
    m.def("type_name", (std::string (module_t::*)() const) &module_t::type);
  }
}


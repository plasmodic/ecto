#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <ecto/name_of.hpp>
#include <map>

#define SHOW() std::cout  << __PRETTY_FUNCTION__ << "\n";

namespace ecto {


  struct connection 
  {
    struct impl_base
    {
      virtual std::string type_name() const = 0;
      virtual void* get() = 0;
      virtual ~impl_base();
      virtual std::string value() = 0;
    };

    template <typename T>
    struct impl : impl_base
    {
      std::string type_name() const
      {
	return ecto::name_of<T>();
      }

      T t;

      void* get() { return &t; }

      std::string value() 
      {
	return boost::lexical_cast<std::string>(t);
      }
    };

    boost::shared_ptr<impl_base> impl_;

    template <typename T>
    static
    connection make()
    {
      connection conn;
      // fixme: allocators
      conn.impl_ = boost::make_shared<impl<T> >();
      return conn;
    }

    std::string type_name() { return impl_->type_name(); }
    std::string value() { return impl_->value(); }

    template <typename T>
    T& get() 
    {
      // fixme: typecheck
      return *(reinterpret_cast<T*>(impl_->get()));
    }

  };

  connection::impl_base::~impl_base() { }


  struct module : boost::noncopyable
  {
    typedef std::map<std::string, connection> connections_t;
    connections_t inputs, outputs;

  };


  template <typename T>
  void wrap(const char* name) 
  {
    boost::python::class_<T, boost::python::bases<module>, boost::shared_ptr<T>, boost::noncopyable> thing(name);
    thing.def("Config", &T::Config);
    thing.def("Process", &T::Process);
  }
      


}

std::string foo() 
{
  ecto::connection cf = ecto::connection::make<float>();
  return cf.type_name();
}

namespace bp = boost::python;


struct OurModule : ecto::module
{
  OurModule() 
  {
    inputs["in"] = ecto::connection::make<int>();
    outputs["out1"] = ecto::connection::make<float>();
    outputs["out2"] = ecto::connection::make<bool>();
  }

  void Config() { }
  void Process() { }

};

struct Generate : ecto::module
{
  int step_, start_;

  void Config(int start, int step)
  {
    SHOW();
    start_ = start;
    step_ = step;
    outputs["out"] = ecto::connection::make<int>();
  }

  void Process() 
  {
    SHOW();
    int& o = outputs["out"].get<int>();
    o = start_ + step_;
    start_ += step_;
  }
};


BOOST_PYTHON_MODULE(ecto)
{
  using namespace ecto;

  bp::def("foo", &foo);

  bp::class_<connection>("Connection", bp::no_init)
    .def("type_name", &connection::type_name)
    .def("value", &connection::value)
    ;

  bp::class_<module::connections_t>("Connections")
    .def(bp::map_indexing_suite<module::connections_t>())
    ;

  bp::class_<module, boost::noncopyable>("Module")
    .def_readwrite("inputs", &module::inputs)
    .def_readwrite("outputs", &module::outputs)
    ;

  wrap<OurModule>("OurModule");
  wrap<Generate>("Generate");
}



#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <ecto/name_of.hpp>
#include <map>
#include <set>

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
    std::string value() 
    { 
      // fixme: smartness
      return impl_->value(); 
    }

    template <typename T>
    T& get() 
    {
      // fixme: typecheck
      return *(reinterpret_cast<T*>(impl_->get()));
    }

    void connect(connection& rhs) 
    {
      impl_ = rhs.impl_;
    }

    bool dirty_;

    bool dirty(bool b) { dirty_ = b; return dirty_; }
    bool dirty() { return dirty_; }
  };

  connection::impl_base::~impl_base() { }


  struct module : boost::noncopyable
  {
    typedef boost::shared_ptr<module> ptr;
    typedef std::map<std::string, connection> connections_t;
    connections_t inputs, outputs;

    std::set<ptr> downstream;
    std::set<module*> upstream;

    void connect(const std::string& out_name, ptr to, const std::string& in_name)
    {
      downstream.insert(to);
      to->upstream.insert(this);
      to->inputs[in_name].connect(outputs[out_name]);
    }

    void dirty() 
    {
      for(connections_t::iterator it = outputs.begin(), end = outputs.end();
	  it != end; 
	  ++it)
	{
	  it->second.dirty(true);
	}
      for (std::set<ptr>::iterator it = downstream.begin(), end = downstream.end();
	   it != end;
	   ++it)
	{
	  (*it)->dirty();
	}
    }
    

  };


  template <typename T>
  void wrap(const char* name) 
  {
    boost::python::class_<T, boost::python::bases<module>, boost::shared_ptr<T>, boost::noncopyable> thing(name);
    thing.def("Config", &T::Config);
    thing.def("Process", &T::Process);
  }
      


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

struct Multiply : ecto::module
{
  int factor_;

  void Config(int factor)
  {
    SHOW();
    factor_ = factor;
    inputs["in"] = ecto::connection::make<int>();
    outputs["out"] = ecto::connection::make<int>();
  }

  void Process() 
  {
    SHOW();
    const int& i = inputs["in"].get<int>();
    int& o = outputs["out"].get<int>();
    o = i * factor_;
  }
};


BOOST_PYTHON_MODULE(ecto)
{
  using namespace ecto;

  bp::class_<connection>("Connection", bp::no_init)
    .def("type_name", &connection::type_name)
    .def("value", &connection::value)
    .def("connect", &connection::connect)
    ;

  bp::class_<module::connections_t>("Connections")
    .def(bp::map_indexing_suite<module::connections_t>())
    ;

  bp::class_<module, boost::noncopyable>("Module")
    .def_readwrite("inputs", &module::inputs)
    .def_readwrite("outputs", &module::outputs)
    .def("connect", &module::connect)
    ;

  wrap<OurModule>("OurModule");
  wrap<Generate>("Generate");
  wrap<Multiply>("Multiply");
}



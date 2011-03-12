#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <ecto/name_of.hpp>
#include <map>
#include <set>
#include <sstream>

#define SHOW() std::cout  << __PRETTY_FUNCTION__ << "\n";

namespace ecto {


  struct connection
  {
    connection():dirty_(true)
    {

    }
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
    bool dirty()const { return dirty_; }
  };

  connection::impl_base::~impl_base() { }


  struct module : boost::noncopyable
  {
    typedef boost::shared_ptr<module> ptr;
    typedef std::map<std::string, connection> connections_t;
    connections_t inputs, outputs;

    virtual ~module(){}
    virtual void Process(){}


    void connect(const std::string& out_name, ptr to, const std::string& in_name)
    {
      to->inputs[in_name].connect(outputs[out_name]);
    }

    void dirty(bool hmm)
    {
      for(connections_t::iterator it = outputs.begin(), end = outputs.end();
          it != end;
          ++it)
        {
          it->second.dirty(hmm);
        }
    }
    bool dirty() const
    {
      for(connections_t::const_iterator it = inputs.begin(), end = inputs.end();
          it != end;
          ++it)
        {
          if(it->second.dirty())
            return true;
        }
      //fixme if there are no inputs assume generator?
      if(inputs.size() == 0)return true;
      return false;
    }


  };


  struct edge
  {
    typedef boost::unordered_set<module::ptr>::iterator iterator;
    typedef boost::unordered_set<module::ptr>::const_iterator const_iterator;
    boost::unordered_set<module::ptr> downstream;
    boost::unordered_set<module::ptr> upstream;
  };

  struct plasm : boost::noncopyable
  {
    typedef boost::unordered_map<module::ptr, edge> map_t;
    map_t edge_map;
    void connect(module::ptr from, const std::string& out_name, module::ptr to, const std::string& in_name)
    {
      edge& f_e = edge_map[from];
      edge& t_e = edge_map[to];
      from->connect(out_name, to, in_name);
      f_e.downstream.insert(to);
      t_e.upstream.insert(from);
    }
    void markDirty(module::ptr m)
    {
      m->dirty(true);
      edge& edges = edge_map[m];
      for (edge::iterator it = edges.downstream.begin(), end = edges.downstream.end(); it != end; ++it)
      {
        (*it)->dirty(true);
      }
    }
    void go(module::ptr m)
    {
      if (!m->dirty())
        return;
      edge& edges = edge_map[m];
      for (edge::iterator it = edges.upstream.begin(), end = edges.upstream.end(); it != end; ++it)
      {
        go(*it);
      }
      m->Process();
      m->dirty(false);
    }
#define PRINT_NAME(_x_i_) typeid(*(_x_i_)).name() << int(0xff & reinterpret_cast<size_t>(&(*(_x_i_))))
    std::string viz() const
    {
      std::stringstream ss;
      for (map_t::const_iterator it = edge_map.begin(), end = edge_map.end(); it != end; ++it)
      {
        ss << PRINT_NAME((it->first)) << " -> {";
        for (edge::const_iterator dit = it->second.downstream.begin(), end = it->second.downstream.end(); dit != end; ++dit)
        {
          ss << PRINT_NAME(*dit);
        }
        ss << "};\n";
      }

      return ss.str();
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
    ;

  wrap<OurModule>("OurModule");
  wrap<Generate>("Generate");
  wrap<Multiply>("Multiply");
}



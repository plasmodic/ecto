#ifndef ECTO_ECTO_HPP_INCLUDED
#define ECTO_ECTO_HPP_INCLUDED

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/noncopyable.hpp>

#include <ecto/util.hpp>

#include <typeinfo>
#include <map>
#include <set>
#include <sstream>

namespace ecto {
  class connection
  {
  public:
    connection();

    template <typename T>
    static
    connection make(const T& t = T(), const std::string& name = std::string(), const std::string& doc = std::string());

    std::string type_name() const;
    std::string value() const;
    std::string name() const;
    std::string doc() const;

    template <typename T>
    const T& get() const;

    template <typename T>
    T& get();
    void connect(connection& rhs);

    inline bool dirty(bool b) { dirty_ = b; return dirty_; }
    inline bool dirty() const { return dirty_; }

  private:
// ############################### NVI ####################################
    struct impl_base
    {
      typedef boost::shared_ptr<impl_base> ptr;
      virtual ~impl_base();
      virtual std::string type_name() const = 0;
      virtual void* get() = 0;
      virtual std::string value() const = 0;
      virtual const std::type_info & type_info() const = 0;

      //convience functions for checking types
      template<typename T>
      static bool inline check(impl_base& i);
      template<typename T>
      static inline void checkThrow(impl_base& i) throw(std::logic_error);
      template<typename T>
      static inline T* get(impl_base& i);

      std::string name,doc;
    };

    template <typename T>
    struct impl : impl_base
    {
      impl(const T& t);
      std::string type_name() const;
      const std::type_info & type_info() const;
      void* get();
      std::string value() const;
      T t;
    };
    connection(impl_base::ptr impl);
    boost::shared_ptr<impl_base> impl_;
    bool dirty_;
  };

  struct module : boost::noncopyable
  {
    typedef boost::shared_ptr<module> ptr;
    typedef std::map<std::string, connection> connections_t;
    module();
    virtual ~module();
    virtual void Process();

    void connect(const std::string& output, ptr to, const std::string& input);
    void dirty(bool hmm);
    bool dirty() const;

    template<typename T>
    connection& setOut(const std::string& name,
                          const std::string& doc = "",
                          const T& t = T());
    template<typename T>
    connection& setIn(const std::string& name,
                          const std::string& doc = "",
                          const T& t = T());
    template<typename T>
    T& getOut(const std::string& name);
    template<typename T>
    const T& getIn(const std::string& name);

    connections_t inputs, outputs;
  private:
    bool dirty_;
    friend class plasm;
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
    void connect(module::ptr from, const std::string& output,
                 module::ptr to, const std::string& input);
    void markDirty(module::ptr m);
    void go(module::ptr m);
    std::string viz() const;
  };

#include <ecto/impl/ecto_impl.hpp>
}//namespace ecto
#endif

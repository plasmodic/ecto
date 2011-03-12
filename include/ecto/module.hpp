#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <ecto/connection.hpp>

#include <map>

namespace ecto {

  struct module : boost::noncopyable
  {
    typedef boost::shared_ptr<module> ptr;
    typedef std::map<std::string, connection> connections_t;
    module();
    virtual ~module();
    virtual void process();

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



#include <ecto/impl/module_impl.hpp>
}//namespace ecto

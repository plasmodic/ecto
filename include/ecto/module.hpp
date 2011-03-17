#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <ecto/tendril.hpp>

#include <map>

namespace ecto 
{
  struct module : boost::noncopyable
  {
    typedef boost::shared_ptr<module> ptr;
    typedef std::map<std::string, tendril> connections_t;
    module();
    virtual ~module();
    virtual void Process();

    void connect(const std::string& output, ptr to, const std::string& input);
    void dirty(bool hmm);
    bool dirty() const;

    template<typename T>
    tendril& 
    setOut(const std::string& name, const std::string& doc = "", const T& t = T());

    template<typename T>
    tendril& 
    setIn(const std::string& name, const std::string& doc = "", const T& t = T());

    template<typename T>
    tendril& 
    setParam(const std::string& name, const std::string& doc = "", const T& t = T());

    template <typename T>
    T& 
    getOut(const std::string& name);
    
    template <typename T>
    const T& 
    getIn(const std::string& name);

    template <typename T>
    const T& 
    getParam(const std::string& name);

    connections_t inputs, outputs, params;

  private:
    bool dirty_;
    friend class plasm;
  };



#include <ecto/impl/module_impl.hpp>
}//namespace ecto

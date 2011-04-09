#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <ecto/tendril.hpp>
#include <ecto/tendrils.hpp>

#include <map>

namespace ecto 
{
  namespace py
  {
    void wrapModule();
  }
  struct module : boost::noncopyable
  {
    typedef boost::shared_ptr<module> ptr;
    typedef tendrils tendrils_t;
    module();
    virtual ~module();
    virtual void Process();
    virtual void Config();


    void connect(const std::string& output, ptr to, const std::string& input);

    void dirty(bool hmm);
    bool dirty() const;


    template<typename T>
       void setOut(const std::string& name,
                       const std::string& doc = "",
                       const T& t = T())
       {
         (outputs).set<T>(name,doc,t);
       }

       template<typename T>
       void setIn(const std::string& name,
                      const std::string& doc = "",
                      const T& t = T())
       {
          (inputs).set<T>(name,doc,t);
       }

       template<typename T>
       void setParam(const std::string& name,
                         const std::string& doc = "",
                         const T& t = T())
       {
         (params).set<T>(name,doc,t);
       }


    template<typename T>
    T& getOut(const std::string& name)
    {
      return (outputs).get<T>(name);
    }

    template<typename T>
    const T& getIn(const std::string& name) const
    {
      return inputs.get<T>(name);
    }

    template<typename T>
    const T& getParam(const std::string& name) const
    {
      return params.get<T>(name);
    }

    virtual std::string name()
    {
      return ecto::name_of(typeid(*this));
    }
    const tendrils& i()const{return inputs;}
    const tendrils& o()const{return outputs;}
    const tendrils& p()const{return params;}
  private:
    tendrils params,inputs,outputs;
    bool dirty_;
    friend class PlasmModule;
    friend class ModuleGraph;
    friend class plasm;
    friend void ecto::py::wrapModule();
  };

}//namespace ecto
